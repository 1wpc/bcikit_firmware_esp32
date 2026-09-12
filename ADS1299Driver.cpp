#include "ADS1299Driver.h"

#include <esp_arduino_version.h>

namespace {

constexpr uint8_t REG_ID = 0x00;
constexpr uint8_t REG_CONFIG1 = 0x01;
constexpr uint8_t REG_CH1SET = 0x05;
constexpr uint8_t REG_GPIO = 0x14;

// ADS1299 specifies a minimum command-decode gap, but the ESP32 also services
// WiFi/BLE interrupts between SPI transactions. A 10-us margin keeps RREG/WREG
// stable without making interactive configuration noticeably slower.
constexpr uint32_t REGISTER_DECODE_DELAY_US = 10;
constexpr uint8_t CONFIGURE_ATTEMPTS = 3;

}  // namespace

ADS1299Driver::ADS1299Driver(SPIClass &spi)
    : spi_(spi),
      spiSettings_(BCIKitConfig::ADS_SPI_CLOCK_HZ, MSBFIRST, SPI_MODE1) {}

bool ADS1299Driver::begin(const ADS1299Settings &settings) {
  pinMode(BCIKitConfig::PIN_ADS_CS, OUTPUT);
  pinMode(BCIKitConfig::PIN_ADS_START, OUTPUT);
  pinMode(BCIKitConfig::PIN_ADS_RESET, OUTPUT);
  pinMode(BCIKitConfig::PIN_ADS_PWDN, OUTPUT);
  // ADS1299 DRDY actively drives the line. GPIO34 is input-only and has no
  // internal pull-up on the classic ESP32, so INPUT is intentional here.
  pinMode(BCIKitConfig::PIN_ADS_DRDY, INPUT);
  pinMode(BCIKitConfig::PIN_ADS_MOSI, OUTPUT);

  digitalWrite(BCIKitConfig::PIN_ADS_CS, HIGH);
  digitalWrite(BCIKitConfig::PIN_ADS_START, LOW);
  digitalWrite(BCIKitConfig::PIN_ADS_PWDN, HIGH);
  digitalWrite(BCIKitConfig::PIN_ADS_RESET, HIGH);
  digitalWrite(BCIKitConfig::PIN_ADS_MOSI, LOW);

  spi_.begin(BCIKitConfig::PIN_ADS_SCK, BCIKitConfig::PIN_ADS_MISO,
             BCIKitConfig::PIN_ADS_MOSI, BCIKitConfig::PIN_ADS_CS);

  if (!startMasterClock()) {
    return false;
  }

  // PWDN is shared by the AFE chain. Cycle it at every ESP32 boot so a board
  // whose physical role switch was changed while power was present cannot
  // retain the previous daisy-chain serial state.
  powerCycle();
  writeCommand(CMD_SDATAC);

  // In a shared-CS/shared-DIN daisy chain, register multiple-readback is not
  // available.  Depending on which DOUT is returned to the host, a direct
  // RREG(ID) can therefore be unavailable even though broadcast WREG and the
  // continuous daisy data path are working.  Keep the ID as a useful
  // single-board diagnostic. Single-board setup uses writable-register
  // verification; multi-board presence is verified from every 27-byte status
  // block once streaming starts.
  readId();
  return reconfigure(settings);
}

bool ADS1299Driver::reconfigure(const ADS1299Settings &settings) {
  if (settings.boardCount < 1 ||
      settings.boardCount > BCIKitConfig::MAX_BOARD_COUNT ||
      (settings.sampleRateHz != 250 && settings.sampleRateHz != 500 &&
       settings.sampleRateHz != 1000) ||
      (settings.profile == ADS1299Profile::InternalTest &&
       settings.testSignalConfig2 != 0xD0 &&
       settings.testSignalConfig2 != 0xD1 &&
       settings.testSignalConfig2 != 0xD4 &&
       settings.testSignalConfig2 != 0xD5)) {
    return false;
  }

  const bool boardCountChanged = settings.boardCount != settings_.boardCount;
  stopContinuous();
  settings_ = settings;
  const RegisterProfile profile = makeProfile(settings_);

  // The daisy chain has one shared RESET/CS/DIN path.  If a host changes its
  // board-count setting shortly after a previous session, a downstream ADS can
  // still be in a shifted RDATAC command state.  A plain WREG then sometimes
  // fails its immediate readback, although the same physical chain succeeds
  // on the next connection.  Reset the whole chain for a board-count change,
  // and retry failed register transactions from that known stopped state.
  // Do not silently accept a failure: callers still receive ApplyFailed when
  // all attempts fail, which preserves detection of an unplugged/bad board.
  for (uint8_t attempt = 0; attempt < CONFIGURE_ATTEMPTS; ++attempt) {
    if (boardCountChanged || attempt != 0) {
      // A RESET pulse only resets registers; it is not sufficient to recover
      // every serial-chain state after the physical ROLE switches or return
      // route have changed. PWDN gives all attached AFEs a clean restart. Use
      // the same full cycle on retries because a first wake can clear the old
      // chain state yet still miss its immediate register-readback window.
      powerCycle();
    }
    writeCommand(CMD_SDATAC);
    delayMicroseconds(REGISTER_DECODE_DELAY_US);
    registerVerified_ = writeAndVerifyProfile(profile);
    if (registerVerified_ && settings_.boardCount > 1) {
      // Do not retain a single-board ID and imply that it identifies every
      // device in a shared multi-board chain.
      deviceId_ = 0x00;
    }
    if (registerVerified_) {
      return true;
    }
  }
  return false;
}

bool ADS1299Driver::startContinuous() {
  if (!registerVerified_) {
    return false;
  }
  // Match the ADS1299 continuous-conversion timing used by the proven legacy
  // firmware: start conversions first, then enable RDATAC. Enabling RDATAC
  // while START is low can leave the first shifted frame misaligned on this
  // board, causing every 27-byte read to miss its 0xC* status header.
  digitalWrite(BCIKitConfig::PIN_ADS_START, LOW);
  delayMicroseconds(10);
  digitalWrite(BCIKitConfig::PIN_ADS_START, HIGH);
  delayMicroseconds(50);
  // RDATAC begins one continuous SPI session.  Keep CS asserted from this
  // command through the subsequent DRDY-driven frame reads.
  select();
  spi_.beginTransaction(spiSettings_);
  spi_.transfer(CMD_RDATAC);
  spi_.endTransaction();
  delayMicroseconds(REGISTER_DECODE_DELAY_US);
  // In RDATAC mode this board requires CS to remain asserted for the whole
  // conversion session.  The known-good legacy firmware does the same: it
  // selects ADS1299 before enabling RDATAC and only releases CS when the
  // stream is stopped.  Toggling CS between DRDY events left the serial data
  // pointer misaligned, so every frame failed the 0xC* status-byte check.
  continuous_ = true;
  return true;
}

void ADS1299Driver::stopContinuous() {
  digitalWrite(BCIKitConfig::PIN_ADS_START, LOW);
  if (continuous_) {
    delayMicroseconds(10);
    writeCommand(CMD_SDATAC);
  }
  continuous_ = false;
}

bool ADS1299Driver::readContinuousFrame(uint8_t *destination, size_t length) {
  if (!continuous_ || destination == nullptr || length == 0 ||
      length > BCIKitConfig::MAX_ADS_FRAME_BYTES) {
    return false;
  }

  spi_.beginTransaction(spiSettings_);
  spi_.transferBytes(nullptr, destination, static_cast<uint32_t>(length));
  spi_.endTransaction();
  return true;
}

uint8_t ADS1299Driver::readId() {
  deviceId_ = readRegister(REG_ID);
  return deviceId_;
}

const char *ADS1299Driver::profileName(ADS1299Profile profile) {
  switch (profile) {
    case ADS1299Profile::EegSrb2:
      return "eeg_srb2";
    case ADS1299Profile::Differential:
      return "differential";
    case ADS1299Profile::InternalTest:
      return "internal_test";
    case ADS1299Profile::InputShort:
      return "input_short";
  }
  return "unknown";
}

bool ADS1299Driver::startMasterClock() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  if (!ledcAttach(BCIKitConfig::PIN_ADS_CLK,
                  BCIKitConfig::ADS_MASTER_CLOCK_HZ, 1)) {
    return false;
  }
  if (!ledcWrite(BCIKitConfig::PIN_ADS_CLK, 1)) {
    return false;
  }
  measuredClockHz_ = ledcReadFreq(BCIKitConfig::PIN_ADS_CLK);
#else
  constexpr uint8_t kLedcChannel = 0;
  measuredClockHz_ = static_cast<uint32_t>(
      ledcSetup(kLedcChannel, BCIKitConfig::ADS_MASTER_CLOCK_HZ, 1));
  ledcAttachPin(BCIKitConfig::PIN_ADS_CLK, kLedcChannel);
  ledcWrite(kLedcChannel, 1);
#endif
  return measuredClockHz_ != 0;
}

void ADS1299Driver::hardwareReset() {
  digitalWrite(BCIKitConfig::PIN_ADS_RESET, LOW);
  delay(1);
  digitalWrite(BCIKitConfig::PIN_ADS_RESET, HIGH);
  delay(10);
}

void ADS1299Driver::powerCycle() {
  digitalWrite(BCIKitConfig::PIN_ADS_START, LOW);
  digitalWrite(BCIKitConfig::PIN_ADS_CS, HIGH);
  digitalWrite(BCIKitConfig::PIN_ADS_RESET, HIGH);
  digitalWrite(BCIKitConfig::PIN_ADS_PWDN, LOW);
  delay(10);
  digitalWrite(BCIKitConfig::PIN_ADS_PWDN, HIGH);
  // Keep the conservative startup delay used by the original initialization;
  // the 2.048-MHz master clock is already running at this point.
  delay(200);
  hardwareReset();
}

void ADS1299Driver::writeCommand(uint8_t command) {
  spi_.beginTransaction(spiSettings_);
  select();
  spi_.transfer(command);
  deselect();
  spi_.endTransaction();
  delayMicroseconds(REGISTER_DECODE_DELAY_US);
}

uint8_t ADS1299Driver::readRegister(uint8_t address) {
  spi_.beginTransaction(spiSettings_);
  select();
  spi_.transfer(static_cast<uint8_t>(CMD_RREG | (address & 0x1F)));
  delayMicroseconds(REGISTER_DECODE_DELAY_US);
  spi_.transfer(0x00);
  delayMicroseconds(REGISTER_DECODE_DELAY_US);
  const uint8_t value = spi_.transfer(0x00);
  deselect();
  spi_.endTransaction();
  delayMicroseconds(REGISTER_DECODE_DELAY_US);
  return value;
}

void ADS1299Driver::writeRegisters(uint8_t startAddress, const uint8_t *values,
                                   size_t count) {
  if (values == nullptr || count == 0) {
    return;
  }

  spi_.beginTransaction(spiSettings_);
  select();
  spi_.transfer(static_cast<uint8_t>(CMD_WREG | (startAddress & 0x1F)));
  delayMicroseconds(REGISTER_DECODE_DELAY_US);
  spi_.transfer(static_cast<uint8_t>(count - 1));
  delayMicroseconds(REGISTER_DECODE_DELAY_US);
  for (size_t i = 0; i < count; ++i) {
    spi_.transfer(values[i]);
  }
  deselect();
  spi_.endTransaction();
  delayMicroseconds(REGISTER_DECODE_DELAY_US);
}

ADS1299Driver::RegisterProfile ADS1299Driver::makeProfile(
    const ADS1299Settings &settings) const {
  RegisterProfile p{};
  p.config1 = settings.sampleRateHz == 1000
                  ? 0x94
                  : (settings.sampleRateHz == 500 ? 0x95 : 0x96);
  // The shipped BCIKit board-to-board return topology has been bench-verified
  // with CONFIG1 bit 6 set for multi-board frames. Keep the single-board reset
  // value unchanged and validate the selected topology end-to-end below.
  if (settings.boardCount > 1) {
    p.config1 |= 0x40;
  }
  p.loff = 0x00;
  p.loffSensp = 0x00;
  p.loffSensn = 0x00;
  p.loffFlip = 0x00;
  p.gpio = 0x0F;
  p.misc1 = 0x00;
  p.misc2 = 0x00;
  p.config4 = 0x00;

  uint8_t channelValue = 0x68;
  switch (settings.profile) {
    case ADS1299Profile::EegSrb2:
      p.config2 = 0xC0;
      p.config3 = 0xEC;
      channelValue = 0x68;
      p.biasSensp = 0xFF;
      p.biasSensn = 0x00;
      break;
    case ADS1299Profile::Differential:
      p.config2 = 0xC0;
      p.config3 = 0xEC;
      channelValue = 0x60;
      p.biasSensp = 0xFF;
      p.biasSensn = 0xFF;
      break;
    case ADS1299Profile::InternalTest:
      p.config2 = settings.testSignalConfig2;
      p.config3 = 0xE0;
      channelValue = 0x65;
      p.biasSensp = 0x00;
      p.biasSensn = 0x00;
      break;
    case ADS1299Profile::InputShort:
      p.config2 = 0xC0;
      p.config3 = 0xE0;
      channelValue = 0x61;
      p.biasSensp = 0x00;
      p.biasSensn = 0x00;
      break;
  }

  for (uint8_t i = 0; i < 8; ++i) {
    p.channel[i] = (settings.channelEnabledMask & (1u << i)) != 0
                       ? channelValue
                       : 0x81;  // powered down, gain 1, input shorted
  }
  p.biasSensp &= settings.channelEnabledMask;
  p.biasSensn &= settings.channelEnabledMask;
  return p;
}

bool ADS1299Driver::writeAndVerifyProfile(const RegisterProfile &p) {
  const uint8_t configBlock[] = {p.config1, p.config2, p.config3, p.loff};
  writeRegisters(REG_CONFIG1, configBlock, sizeof(configBlock));

  uint8_t channelAndBiasBlock[13]{};
  memcpy(channelAndBiasBlock, p.channel, sizeof(p.channel));
  channelAndBiasBlock[8] = p.biasSensp;
  channelAndBiasBlock[9] = p.biasSensn;
  channelAndBiasBlock[10] = p.loffSensp;
  channelAndBiasBlock[11] = p.loffSensn;
  channelAndBiasBlock[12] = p.loffFlip;
  writeRegisters(REG_CH1SET, channelAndBiasBlock, sizeof(channelAndBiasBlock));

  const uint8_t tailBlock[] = {p.gpio, p.misc1, p.misc2, p.config4};
  writeRegisters(REG_GPIO, tailBlock, sizeof(tailBlock));

  // With shared CS/DIN and a chained DOUT, individual RREG results are not a
  // reliable multi-board configuration gate. The caller verifies multi-board
  // operation using the chained RDATAC data path instead.
  if (settings_.boardCount > 1) {
    return true;
  }

  // CONFIG1.DAISY_EN=0 is the final acquisition setting, but this board's
  // CHAIN_MISO_RETURN route can expose the daisy input rather than the local
  // register byte during RREG. Temporarily select ADS1299 multiple-readback
  // mode (bit 6 = 1), verify the local device and cache its ID, then restore
  // the requested daisy acquisition setting before returning.
  const uint8_t readableConfig1 = static_cast<uint8_t>(p.config1 | 0x40);
  writeRegisters(REG_CONFIG1, &readableConfig1, 1);

  bool ok = true;
  ok &= verifyRegister(0x01, readableConfig1, 0x67);
  ok &= verifyRegister(0x02, p.config2, 0x17);
  ok &= verifyRegister(0x03, p.config3, 0xFC);
  ok &= verifyRegister(0x04, p.loff, 0xFF);
  for (uint8_t i = 0; i < 8; ++i) {
    ok &= verifyRegister(static_cast<uint8_t>(0x05 + i), p.channel[i], 0xFF);
  }
  ok &= verifyRegister(0x0D, p.biasSensp, 0xFF);
  ok &= verifyRegister(0x0E, p.biasSensn, 0xFF);
  ok &= verifyRegister(0x0F, p.loffSensp, 0xFF);
  ok &= verifyRegister(0x10, p.loffSensn, 0xFF);
  ok &= verifyRegister(0x11, p.loffFlip, 0xFF);
  ok &= verifyRegister(0x14, p.gpio, 0xFF);
  ok &= verifyRegister(0x15, p.misc1, 0x20);
  ok &= verifyRegister(0x16, p.misc2, 0x00);
  ok &= verifyRegister(0x17, p.config4, 0x0F);
  deviceId_ = readRegister(REG_ID);
  ok &= deviceId_ == 0x3E;
  writeRegisters(REG_CONFIG1, &p.config1, 1);
  return ok;
}

bool ADS1299Driver::verifyRegister(uint8_t address, uint8_t expected,
                                  uint8_t writableMask) {
  const uint8_t actual = readRegister(address);
  return (actual & writableMask) == (expected & writableMask);
}

void ADS1299Driver::select() {
  digitalWrite(BCIKitConfig::PIN_ADS_CS, LOW);
}

void ADS1299Driver::deselect() {
  digitalWrite(BCIKitConfig::PIN_ADS_CS, HIGH);
}
