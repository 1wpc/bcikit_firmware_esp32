#pragma once

#include <Arduino.h>
#include <SPI.h>

#include "BCIKitConfig.h"

enum class ADS1299Profile : uint8_t {
  EegSrb2,
  Differential,
  InternalTest,
  InputShort,
};

struct ADS1299Settings {
  uint8_t boardCount = BCIKitConfig::DEFAULT_BOARD_COUNT;
  uint16_t sampleRateHz = BCIKitConfig::DEFAULT_SAMPLE_RATE_HZ;
  ADS1299Profile profile = ADS1299Profile::EegSrb2;
  uint8_t channelEnabledMask = 0xFF;
  uint8_t testSignalConfig2 = 0xD0;
};

class ADS1299Driver {
 public:
  explicit ADS1299Driver(SPIClass &spi);

  bool begin(const ADS1299Settings &settings);
  bool reconfigure(const ADS1299Settings &settings);
  bool startContinuous();
  void stopContinuous();
  bool readContinuousFrame(uint8_t *destination, size_t length);
  void invalidateConfiguration() { registerVerified_ = false; }

  uint8_t readId();
  uint8_t cachedId() const { return deviceId_; }
  bool registerVerified() const { return registerVerified_; }
  uint32_t measuredClockHz() const { return measuredClockHz_; }
  const ADS1299Settings &settings() const { return settings_; }

  static const char *profileName(ADS1299Profile profile);

 private:
  enum Command : uint8_t {
    CMD_START = 0x08,
    CMD_STOP = 0x0A,
    CMD_RDATAC = 0x10,
    CMD_SDATAC = 0x11,
    CMD_RREG = 0x20,
    CMD_WREG = 0x40,
  };

  struct RegisterProfile {
    uint8_t config1;
    uint8_t config2;
    uint8_t config3;
    uint8_t loff;
    uint8_t channel[8];
    uint8_t biasSensp;
    uint8_t biasSensn;
    uint8_t loffSensp;
    uint8_t loffSensn;
    uint8_t loffFlip;
    uint8_t gpio;
    uint8_t misc1;
    uint8_t misc2;
    uint8_t config4;
  };

  bool startMasterClock();
  void powerCycle();
  void hardwareReset();
  void writeCommand(uint8_t command);
  uint8_t readRegister(uint8_t address);
  void writeRegisters(uint8_t startAddress, const uint8_t *values, size_t count);
  RegisterProfile makeProfile(const ADS1299Settings &settings) const;
  bool writeAndVerifyProfile(const RegisterProfile &profile);
  bool verifyRegister(uint8_t address, uint8_t expected, uint8_t writableMask);
  void select();
  void deselect();

  SPIClass &spi_;
  SPISettings spiSettings_;
  ADS1299Settings settings_;
  bool registerVerified_ = false;
  bool continuous_ = false;
  uint32_t measuredClockHz_ = 0;
  uint8_t deviceId_ = 0;
};
