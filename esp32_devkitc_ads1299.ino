#include <Arduino.h>
#include <SPI.h>
#include <esp_timer.h>
#include <cstdlib>
#include <cstring>

#include "ADS1299Driver.h"
#include "BCIKitBLE.h"
#include "BCIKitConfig.h"
#include "BCIKitProtocol.h"
#include "BCIKitWiFi.h"
#include "OpenBCIProtocol.h"

namespace {

struct AcquisitionStats {
  uint32_t drdyCount = 0;
  uint32_t frameCount = 0;
  uint32_t frameSyncError = 0;
  uint32_t spiError = 0;
  uint32_t drdyTimeout = 0;
  uint32_t queueOverrun = 0;
  uint32_t transportDrop = 0;
  uint32_t registerVerifyError = 0;
};

// Use the classic ESP32's user-accessible VSPI controller. FSPI/SPI1 is tied
// to the module flash and must never be used for the ADS1299 bus.
SPIClass adsSpi(VSPI);
ADS1299Driver ads1299(adsSpi);
BCIKitWiFi wifiTransport;
BCIKitBLE bleTransport;
ADS1299Settings activeSettings;
volatile BCIKitProtocol::OutputFormat outputFormat =
    BCIKitProtocol::OutputFormat::BCIKitRawV1;

QueueHandle_t drdyQueue = nullptr;
QueueHandle_t sampleQueue = nullptr;
SemaphoreHandle_t serialWriteMutex = nullptr;
TaskHandle_t acquisitionTaskHandle = nullptr;
TaskHandle_t transportTaskHandle = nullptr;

volatile bool acquiring = false;
volatile bool droppedBeforePending = false;
portMUX_TYPE statsMux = portMUX_INITIALIZER_UNLOCKED;
AcquisitionStats stats;
uint32_t nextSequence = 0;
uint32_t acquisitionEpoch = 0;

String commandLine;
uint32_t lastAsciiByteMs = 0;
BCIKitProtocol::ControlParser serialControlParser;
BCIKitProtocol::ControlParser wifiControlParser;
BCIKitProtocol::ControlParser bleControlParser;

enum class ControlTransport : uint8_t { Serial = 0, WiFiTcp = 1, Ble = 2 };

void markDroppedBefore() {
  portENTER_CRITICAL(&statsMux);
  droppedBeforePending = true;
  portEXIT_CRITICAL(&statsMux);
}

bool consumeDroppedBefore() {
  portENTER_CRITICAL(&statsMux);
  const bool result = droppedBeforePending;
  droppedBeforePending = false;
  portEXIT_CRITICAL(&statsMux);
  return result;
}

void IRAM_ATTR onAdsDrdy() {
  const uint64_t timestampUs = static_cast<uint64_t>(esp_timer_get_time());
  BaseType_t higherPriorityTaskWoken = pdFALSE;

  portENTER_CRITICAL_ISR(&statsMux);
  ++stats.drdyCount;
  portEXIT_CRITICAL_ISR(&statsMux);

  if (xQueueSendFromISR(drdyQueue, &timestampUs, &higherPriorityTaskWoken) !=
      pdTRUE) {
    portENTER_CRITICAL_ISR(&statsMux);
    ++stats.queueOverrun;
    droppedBeforePending = true;
    portEXIT_CRITICAL_ISR(&statsMux);
  }
  if (higherPriorityTaskWoken == pdTRUE) {
    portYIELD_FROM_ISR();
  }
}

bool frameStatusValid(const uint8_t *frame, uint8_t boardCount) {
  for (uint8_t board = 0; board < boardCount; ++board) {
    const size_t offset = board * BCIKitConfig::BYTES_PER_BOARD;
    if ((frame[offset] & 0xF0) != 0xC0) {
      return false;
    }
  }
  return true;
}

uint16_t modeFlags() {
  uint16_t flags = BCIKitProtocol::FLAG_TIME_VALID;
  if (ads1299.registerVerified()) {
    flags |= BCIKitProtocol::FLAG_REGISTER_VERIFIED;
  }
  if (activeSettings.profile == ADS1299Profile::InternalTest) {
    flags |= BCIKitProtocol::FLAG_TEST_SIGNAL;
  }
  if (activeSettings.profile == ADS1299Profile::InputShort) {
    flags |= BCIKitProtocol::FLAG_INPUT_SHORT;
  }
  return flags;
}

void stopAcquisition() {
  if (!acquiring) {
    return;
  }
  acquiring = false;
  detachInterrupt(digitalPinToInterrupt(BCIKitConfig::PIN_ADS_DRDY));
  ads1299.stopContinuous();
  xQueueReset(drdyQueue);
  xQueueReset(sampleQueue);
}

bool startAcquisition() {
  if (acquiring || !ads1299.registerVerified()) {
    return acquiring;
  }
  if (outputFormat == BCIKitProtocol::OutputFormat::OpenBCICyton &&
      activeSettings.boardCount != 1) {
    return false;
  }
  // At 115200 baud, native BCIKit frames (55 bytes * 250 SPS for one
  // board) exceed the UART's usable payload bandwidth. Without a configured
  // WiFi target, only the standard 33-byte OpenBCI stream at 250 SPS is safe.
  const bool hasBinaryTransport = wifiTransport.targetConfigured() ||
                                  bleTransport.sessionActive();
  if (!hasBinaryTransport &&
      (outputFormat != BCIKitProtocol::OutputFormat::OpenBCICyton ||
       activeSettings.sampleRateHz != 250)) {
    return false;
  }
  // BLE V1 has no retransmission or credit protocol. Keep its promised
  // operating envelope below the phone-dependent notification throughput.
  if (bleTransport.sessionActive() &&
      (activeSettings.boardCount > 2 ||
       (activeSettings.boardCount == 2 && activeSettings.sampleRateHz > 250))) {
    return false;
  }

  xQueueReset(drdyQueue);
  xQueueReset(sampleQueue);
  portENTER_CRITICAL(&statsMux);
  ++acquisitionEpoch;
  portEXIT_CRITICAL(&statsMux);
  acquiring = ads1299.startContinuous();
  if (!acquiring) {
    return false;
  }
  // Arm DRDY only after the ADC has entered RDATAC mode, so an edge from the
  // START/RDATAC transition cannot be mistaken for a complete data frame.
  attachInterrupt(digitalPinToInterrupt(BCIKitConfig::PIN_ADS_DRDY), onAdsDrdy,
                  FALLING);
  return acquiring;
}

void acquisitionTask(void *) {
  uint32_t observedEpoch = 0;
  uint8_t framesToDiscard = 0;
  uint8_t consecutiveSyncErrors = 0;
  uint8_t consecutiveTimeouts = 0;

  for (;;) {
    portENTER_CRITICAL(&statsMux);
    const uint32_t currentEpoch = acquisitionEpoch;
    portEXIT_CRITICAL(&statsMux);
    if (observedEpoch != currentEpoch) {
      observedEpoch = currentEpoch;
      framesToDiscard = BCIKitConfig::INITIAL_FRAMES_TO_DISCARD;
      consecutiveSyncErrors = 0;
      consecutiveTimeouts = 0;
    }

    uint64_t timestampUs = 0;
    const uint32_t periodMs = max<uint32_t>(1, 1000 / activeSettings.sampleRateHz);
    const TickType_t timeout = pdMS_TO_TICKS(periodMs * 3 + 2);

    if (xQueueReceive(drdyQueue, &timestampUs, timeout) != pdTRUE) {
      if (acquiring) {
        portENTER_CRITICAL(&statsMux);
        ++stats.drdyTimeout;
        portEXIT_CRITICAL(&statsMux);
        if (++consecutiveTimeouts >= 3) {
          stopAcquisition();
        }
      }
      continue;
    }
    if (!acquiring) {
      continue;
    }
    consecutiveTimeouts = 0;

    BCIKitProtocol::SampleRecord sample{};
    sample.sequence = nextSequence++;
    sample.timestampUs = timestampUs;
    sample.flags = modeFlags();
    sample.sampleRateHz = activeSettings.sampleRateHz;
    sample.boardCount = activeSettings.boardCount;
    sample.gainCode = 6;  // ADS1299 gain 24 encoding.
    sample.payloadLength = static_cast<uint16_t>(
        activeSettings.boardCount * BCIKitConfig::BYTES_PER_BOARD);

    if (!ads1299.readContinuousFrame(sample.payload, sample.payloadLength)) {
      portENTER_CRITICAL(&statsMux);
      ++stats.spiError;
      portEXIT_CRITICAL(&statsMux);
      markDroppedBefore();
      continue;
    }

    if (!frameStatusValid(sample.payload, sample.boardCount)) {
      portENTER_CRITICAL(&statsMux);
      ++stats.frameSyncError;
      portEXIT_CRITICAL(&statsMux);
      ++consecutiveSyncErrors;
      markDroppedBefore();
      if (consecutiveSyncErrors >= 3) {
        // Stop safely. The user can inspect wiring/board_count and issue start.
        ads1299.invalidateConfiguration();
        stopAcquisition();
      }
      continue;
    }
    consecutiveSyncErrors = 0;

    if (framesToDiscard > 0) {
      --framesToDiscard;
      continue;
    }
    if (consumeDroppedBefore()) {
      sample.flags |= BCIKitProtocol::FLAG_DROPPED_BEFORE;
    }

    if (xQueueSend(sampleQueue, &sample, 0) != pdTRUE) {
      portENTER_CRITICAL(&statsMux);
      ++stats.queueOverrun;
      portEXIT_CRITICAL(&statsMux);
      markDroppedBefore();
      continue;
    }

    portENTER_CRITICAL(&statsMux);
    ++stats.frameCount;
    portEXIT_CRITICAL(&statsMux);
  }
}

void transportTask(void *) {
  // A one-board SAMPLE wire frame is 55 bytes. Sending it at 250 SPS as one
  // Notify per frame overwhelms some desktop BLE controllers before their byte
  // bandwidth is exhausted. The BLE profile defines each characteristic as a
  // byte stream, so coalesce adjacent complete frames into one ATT value.
  // Keep a 16-channel notification at two raw frames (164 bytes). Although a
  // larger negotiated ATT MTU can carry four frames, CoreBluetooth/NimBLE can
  // block inside a 328-byte notify while controller credits are exhausted,
  // starving this task and overflowing the sample queue.
  constexpr size_t kBleBatchCapacity = 512;
  constexpr uint8_t kMaxBleBatchSamples = 2;
  // At 250 SPS the next frame arrives in about 4 ms.  Waiting briefly lets a
  // batch form even when this task caught up with the producer queue, while
  // keeping added first-frame latency bounded to roughly 4 ms (two frames).
  constexpr TickType_t kBleBatchWait = pdMS_TO_TICKS(5);
  uint8_t bleBatch[kBleBatchCapacity];
  uint8_t encoded[BCIKitConfig::MAX_PROTOCOL_FRAME_BYTES];
  for (;;) {
    BCIKitProtocol::SampleRecord sample{};
    if (xQueueReceive(sampleQueue, &sample, portMAX_DELAY) != pdTRUE) {
      continue;
    }
    if (bleTransport.sessionActive() &&
        outputFormat == BCIKitProtocol::OutputFormat::BCIKitRawV1) {
      size_t batchLength = 0;
      uint8_t batchSamples = 0;
      const size_t frameCapacity = 28 + sample.payloadLength;
      const uint8_t maxBatchSamples =
          frameCapacity == 0 ? 1
                             : static_cast<uint8_t>(
                                   (kBleBatchCapacity / frameCapacity) <
                                           kMaxBleBatchSamples
                                       ? (kBleBatchCapacity / frameCapacity)
                                       : kMaxBleBatchSamples);
      do {
        const size_t length =
            BCIKitProtocol::encodeSample(sample, encoded, sizeof(encoded));
        if (length == 0 || length > kBleBatchCapacity - batchLength) {
          // A single frame never exceeds the BLE payload in the supported
          // 1/2-board modes. Do not corrupt the byte stream on an unexpected
          // encode or size failure.
          portENTER_CRITICAL(&statsMux);
          ++stats.transportDrop;
          portEXIT_CRITICAL(&statsMux);
          markDroppedBefore();
        } else {
          memcpy(bleBatch + batchLength, encoded, length);
          batchLength += length;
          ++batchSamples;
        }
      } while (batchSamples < maxBatchSamples &&
               xQueueReceive(sampleQueue, &sample, kBleBatchWait) == pdTRUE);

      if (batchLength != 0 &&
          !bleTransport.sendSamplePacket(bleBatch, batchLength)) {
        portENTER_CRITICAL(&statsMux);
        stats.transportDrop += batchSamples;
        portEXIT_CRITICAL(&statsMux);
        for (uint8_t i = 0; i < batchSamples; ++i) {
          markDroppedBefore();
        }
      }
      continue;
    }
    const BCIKitProtocol::OutputFormat format = outputFormat;
    const size_t length =
        format == BCIKitProtocol::OutputFormat::BCIKitRawV1
            ? BCIKitProtocol::encodeSample(sample, encoded, sizeof(encoded))
            : OpenBCIProtocol::encodeCytonPacket(sample, encoded,
                                                 sizeof(encoded));
    bool transportOk = length != 0;
    if (transportOk && bleTransport.sessionActive()) {
      transportOk = bleTransport.sendSamplePacket(encoded, length);
    } else if (transportOk && wifiTransport.targetConfigured()) {
      // Once a GUI/host configures /tcp or /udp, samples use WiFi only. UART
      // remains a readable control/log console instead of being flooded with
      // binary frames.
      transportOk = wifiTransport.sendPacket(encoded, length);
    } else if (transportOk &&
               xSemaphoreTake(serialWriteMutex, portMAX_DELAY) == pdTRUE) {
      transportOk = Serial.write(encoded, length) == length;
      xSemaphoreGive(serialWriteMutex);
    } else {
      transportOk = false;
    }
    if (!transportOk) {
      portENTER_CRITICAL(&statsMux);
      ++stats.transportDrop;
      portEXIT_CRITICAL(&statsMux);
      markDroppedBefore();
    }
  }
}

AcquisitionStats statsSnapshot() {
  portENTER_CRITICAL(&statsMux);
  const AcquisitionStats snapshot = stats;
  portEXIT_CRITICAL(&statsMux);
  return snapshot;
}

void resetAcquisitionStats() {
  portENTER_CRITICAL(&statsMux);
  stats = AcquisitionStats{};
  droppedBeforePending = false;
  portEXIT_CRITICAL(&statsMux);
}

bool writeSerialBytes(const uint8_t *data, size_t length) {
  if (data == nullptr || length == 0 ||
      xSemaphoreTake(serialWriteMutex, portMAX_DELAY) != pdTRUE) {
    return false;
  }
  const size_t written = Serial.write(data, length);
  xSemaphoreGive(serialWriteMutex);
  return written == length;
}

BCIKitProtocol::DeviceInfo deviceInfoSnapshot() {
  BCIKitProtocol::DeviceInfo info{};
  info.firmwareMajor = 0;
  info.firmwareMinor = 7;
  info.firmwarePatch = 12;
  info.hardwareType = 1;  // ESP32-DevKitC host + BCIKit AFE8.
  info.capabilities =
      BCIKitProtocol::CAP_BCIKIT_RAW_V1 |
      BCIKitProtocol::CAP_OPENBCI_CYTON |
      BCIKitProtocol::CAP_MULTI_BOARD |
      BCIKitProtocol::CAP_DRDY_TIMESTAMP | BCIKitProtocol::CAP_CRC16 |
      BCIKitProtocol::CAP_INTERNAL_TEST |
      BCIKitProtocol::CAP_INPUT_SHORT |
      BCIKitProtocol::CAP_CHANNEL_MASK |
      BCIKitProtocol::CAP_BINARY_CONTROL |
      BCIKitProtocol::CAP_WIFI_OPENBCI_API |
      BCIKitProtocol::CAP_WIFI_BCIKIT_STREAM |
      BCIKitProtocol::CAP_WIFI_BCIKIT_CONTROL |
      BCIKitProtocol::CAP_AUTO_PROTOCOL_SESSION |
      BCIKitProtocol::CAP_BLE_BCIKIT_STREAM |
      BCIKitProtocol::CAP_BLE_BCIKIT_CONTROL;
  info.maxBoardCount = BCIKitConfig::MAX_BOARD_COUNT;
  info.sampleRateMask = 0x03;   // bit0=250 SPS, bit1=500 SPS.
  info.outputFormatMask = 0x03; // bit0=BCIKit, bit1=OpenBCI Cyton.
  info.serialBaud = BCIKitConfig::SERIAL_BAUD;
  info.deviceId = ESP.getEfuseMac();
  return info;
}

BCIKitProtocol::DeviceStatus deviceStatusSnapshot() {
  const AcquisitionStats s = statsSnapshot();
  BCIKitProtocol::DeviceStatus result{};
  result.acquisitionState = acquiring ? 1 : 0;
  result.outputFormat = outputFormat;
  result.profile = static_cast<uint8_t>(activeSettings.profile);
  result.boardCount = activeSettings.boardCount;
  result.sampleRateHz = activeSettings.sampleRateHz;
  result.gainCode = 6;
  result.channelEnabledMask = activeSettings.channelEnabledMask;
  result.registerVerified = ads1299.registerVerified() ? 1 : 0;
  result.adsId = ads1299.cachedId();
  result.nextSequence = nextSequence;
  result.uptimeUs = static_cast<uint64_t>(esp_timer_get_time());
  result.counters[0] = s.drdyCount;
  result.counters[1] = s.frameCount;
  result.counters[2] = s.frameSyncError;
  result.counters[3] = s.spiError;
  result.counters[4] = s.drdyTimeout;
  result.counters[5] = s.queueOverrun;
  result.counters[6] = s.transportDrop;
  result.counters[7] = s.registerVerifyError;
  return result;
}

bool writeControlBytes(ControlTransport transport, const uint8_t *data,
                       size_t length) {
  if (transport == ControlTransport::WiFiTcp) {
    return wifiTransport.sendControlPacket(data, length);
  }
  if (transport == ControlTransport::Ble) {
    return bleTransport.sendControlPacket(data, length);
  }
  return writeSerialBytes(data, length);
}

void sendAck(ControlTransport transport, uint32_t requestId,
             BCIKitProtocol::CommandId command,
             BCIKitProtocol::CommandStatus status,
             const uint8_t *payload = nullptr, size_t payloadLength = 0) {
  uint8_t frame[64]{};
  const size_t length = BCIKitProtocol::encodeControlFrame(
      BCIKitProtocol::FrameType::Ack, requestId,
      static_cast<uint8_t>(command), status, payload, payloadLength, frame,
      sizeof(frame));
  if (!writeControlBytes(transport, frame, length)) {
    portENTER_CRITICAL(&statsMux);
    ++stats.transportDrop;
    portEXIT_CRITICAL(&statsMux);
  }
}

void sendDeviceInfo(ControlTransport transport, uint32_t requestId) {
  uint8_t frame[64]{};
  const size_t length = BCIKitProtocol::encodeDeviceInfo(
      requestId, deviceInfoSnapshot(), frame, sizeof(frame));
  if (!writeControlBytes(transport, frame, length)) {
    portENTER_CRITICAL(&statsMux);
    ++stats.transportDrop;
    portEXIT_CRITICAL(&statsMux);
  }
}

void sendDeviceStatus(ControlTransport transport, uint32_t requestId) {
  uint8_t frame[80]{};
  const size_t length = BCIKitProtocol::encodeDeviceStatus(
      requestId, deviceStatusSnapshot(), frame, sizeof(frame));
  if (!writeControlBytes(transport, frame, length)) {
    portENTER_CRITICAL(&statsMux);
    ++stats.transportDrop;
    portEXIT_CRITICAL(&statsMux);
  }
}

bool updateSettings(const ADS1299Settings &requested) {
  if (acquiring) {
    return false;
  }
  const bool unchanged =
      requested.boardCount == activeSettings.boardCount &&
      requested.sampleRateHz == activeSettings.sampleRateHz &&
      requested.profile == activeSettings.profile &&
      requested.channelEnabledMask == activeSettings.channelEnabledMask &&
      requested.testSignalConfig2 == activeSettings.testSignalConfig2;
  // Desktop handshakes intentionally resend the complete desired
  // configuration on every connection.  Treat an already-applied and still
  // verified setting as idempotent instead of resetting and rewriting the
  // ADS1299 chain on every reconnect.
  if (unchanged && ads1299.registerVerified()) {
    return true;
  }
  const ADS1299Settings previous = activeSettings;
  if (ads1299.reconfigure(requested)) {
    activeSettings = requested;
    return true;
  }

  portENTER_CRITICAL(&statsMux);
  ++stats.registerVerifyError;
  portEXIT_CRITICAL(&statsMux);
  ads1299.reconfigure(previous);
  return false;
}

void printHelp() {
  Serial.println("BCIKit commands (line ending: Newline):");
  Serial.println("  info                   show device capabilities");
  Serial.println("  status                 show configuration and counters");
  Serial.println("  output bcikit|openbci  select stream protocol");
  Serial.println("  start                  start selected binary stream");
  Serial.println("  stop                   stop binary stream");
  Serial.println("  rate 250|500           set sample rate while stopped");
  Serial.println("  boards 1|2|3|4         set daisy-chain board count");
  Serial.println("  mode eeg|diff|test|short");
  Serial.println("  channels 0xFF          set enabled-channel bit mask");
  Serial.println("  id                     read ADS1299 ID while stopped");
  Serial.println("  crc                    run CRC self-test");
  Serial.println("  resetstats             clear acquisition counters");
  Serial.println("  wifi                   show WiFi Direct status");
  Serial.println("  help");
}

void printStatus() {
  const AcquisitionStats s = statsSnapshot();
  Serial.printf(
      "state=%s output=%s profile=%s rate=%u boards=%u channels=%u mask=0x%02X\n",
                acquiring ? "STREAMING" : "READY",
                outputFormat == BCIKitProtocol::OutputFormat::BCIKitRawV1
                    ? "bcikit"
                    : "openbci",
                ADS1299Driver::profileName(activeSettings.profile),
                activeSettings.sampleRateHz, activeSettings.boardCount,
                activeSettings.boardCount * 8,
                activeSettings.channelEnabledMask);
  Serial.printf("ads_id=0x%02X register_verified=%s clk_requested=%lu clk_api=%lu\n",
                ads1299.cachedId(), ads1299.registerVerified() ? "yes" : "no",
                static_cast<unsigned long>(BCIKitConfig::ADS_MASTER_CLOCK_HZ),
                static_cast<unsigned long>(ads1299.measuredClockHz()));
  Serial.printf(
      "drdy=%lu frames=%lu sync_error=%lu spi_error=%lu timeout=%lu "
      "queue_overrun=%lu transport_drop=%lu verify_error=%lu\n",
      static_cast<unsigned long>(s.drdyCount),
      static_cast<unsigned long>(s.frameCount),
      static_cast<unsigned long>(s.frameSyncError),
      static_cast<unsigned long>(s.spiError),
      static_cast<unsigned long>(s.drdyTimeout),
      static_cast<unsigned long>(s.queueOverrun),
      static_cast<unsigned long>(s.transportDrop),
      static_cast<unsigned long>(s.registerVerifyError));
  Serial.printf(
      "wifi_ssid=%s wifi_ip=%s wifi_protocol=%u wifi_session=%s "
      "wifi_target=%s\n",
                wifiTransport.name().c_str(),
                wifiTransport.ipAddress().c_str(),
                static_cast<unsigned>(wifiTransport.protocol()),
                wifiTransport.sessionName().c_str(),
                wifiTransport.targetConnected() ? "connected" : "none");
  Serial.printf("ble_name=%s ble_connected=%s ble_mtu=%u ble_payload=%u "
                "ble_interval=%.2fms\n",
                bleTransport.name().c_str(),
                bleTransport.connected() ? "yes" : "no", bleTransport.peerMtu(),
                static_cast<unsigned>(bleTransport.samplePacketCapacity()),
                bleTransport.connectionIntervalUnits() * 1.25f);
}

void printInfo() {
  const BCIKitProtocol::DeviceInfo info = deviceInfoSnapshot();
  Serial.printf(
      "device=BCIKit-AFE8 host=ESP32-DevKitC firmware=%u.%u.%u "
      "protocol=%u id=%08lX%08lX\n",
      info.firmwareMajor, info.firmwareMinor, info.firmwarePatch,
      BCIKitProtocol::PROTOCOL_VERSION,
      static_cast<unsigned long>(info.deviceId >> 32),
      static_cast<unsigned long>(info.deviceId));
  Serial.println(
      "capabilities=bcikit_raw_v1,openbci_cyton_8ch,binary_control,"
      "wifi_tcp_udp_ssdp,wifi_bcikit_control,auto_protocol_session,"
      "ble_bcikit_stream,ble_bcikit_control,250_500_sps,1_4_boards,"
      "channel_mask,test,short");
}

void processControlRequest(const BCIKitProtocol::ControlRequest &request,
                           ControlTransport transport) {
  using BCIKitProtocol::CommandId;
  using BCIKitProtocol::CommandStatus;

  auto reply = [&](CommandStatus status, const uint8_t *payload = nullptr,
                   size_t payloadLength = 0) {
    sendAck(transport, request.requestId, request.command, status, payload,
            payloadLength);
  };

  auto requireLength = [&](uint16_t expected) {
    if (request.payloadLength != expected) {
      reply(CommandStatus::InvalidLength);
      return false;
    }
    return true;
  };

  switch (request.command) {
    case CommandId::GetInfo:
      if (requireLength(0)) {
        sendDeviceInfo(transport, request.requestId);
      }
      return;
    case CommandId::GetStatus:
      if (requireLength(0)) {
        sendDeviceStatus(transport, request.requestId);
      }
      return;
    case CommandId::Start:
      if (!requireLength(0)) {
        return;
      }
      reply(startAcquisition() ? CommandStatus::Ok
                               : CommandStatus::ApplyFailed);
      return;
    case CommandId::Stop:
      if (!requireLength(0)) {
        return;
      }
      stopAcquisition();
      reply(CommandStatus::Ok);
      return;
    case CommandId::SetOutputFormat: {
      if (!requireLength(1)) {
        return;
      }
      if (acquiring) {
        reply(CommandStatus::Busy);
        return;
      }
      if (request.payload[0] >
          static_cast<uint8_t>(BCIKitProtocol::OutputFormat::OpenBCICyton)) {
        reply(CommandStatus::InvalidValue);
        return;
      }
      const auto requested =
          static_cast<BCIKitProtocol::OutputFormat>(request.payload[0]);
      if (transport == ControlTransport::Ble &&
          requested != BCIKitProtocol::OutputFormat::BCIKitRawV1) {
        reply(CommandStatus::Unsupported);
        return;
      }
      if ((transport == ControlTransport::WiFiTcp &&
           requested != BCIKitProtocol::OutputFormat::BCIKitRawV1) ||
          (requested == BCIKitProtocol::OutputFormat::OpenBCICyton &&
           activeSettings.boardCount != 1)) {
        reply(CommandStatus::InvalidValue);
        return;
      }
      outputFormat = requested;
      reply(CommandStatus::Ok);
      return;
    }
    case CommandId::SetSampleRate: {
      if (!requireLength(2)) {
        return;
      }
      if (acquiring) {
        reply(CommandStatus::Busy);
        return;
      }
      const uint16_t rate = static_cast<uint16_t>(request.payload[0]) |
                            static_cast<uint16_t>(request.payload[1]) << 8;
      if (rate != 250 && rate != 500) {
        reply(CommandStatus::InvalidValue);
        return;
      }
      ADS1299Settings requested = activeSettings;
      requested.sampleRateHz = rate;
      reply(updateSettings(requested) ? CommandStatus::Ok
                                      : CommandStatus::ApplyFailed);
      return;
    }
    case CommandId::SetBoardCount: {
      if (!requireLength(1)) {
        return;
      }
      if (acquiring) {
        reply(CommandStatus::Busy);
        return;
      }
      const uint8_t count = request.payload[0];
      if (count < 1 || count > BCIKitConfig::MAX_BOARD_COUNT ||
          (outputFormat == BCIKitProtocol::OutputFormat::OpenBCICyton &&
           count != 1)) {
        reply(CommandStatus::InvalidValue);
        return;
      }
      ADS1299Settings requested = activeSettings;
      requested.boardCount = count;
      reply(updateSettings(requested) ? CommandStatus::Ok
                                      : CommandStatus::ApplyFailed);
      return;
    }
    case CommandId::SetProfile: {
      if (!requireLength(1)) {
        return;
      }
      if (acquiring) {
        reply(CommandStatus::Busy);
        return;
      }
      if (request.payload[0] >
          static_cast<uint8_t>(ADS1299Profile::InputShort)) {
        reply(CommandStatus::InvalidValue);
        return;
      }
      ADS1299Settings requested = activeSettings;
      requested.profile = static_cast<ADS1299Profile>(request.payload[0]);
      if (requested.profile == ADS1299Profile::InternalTest) {
        requested.testSignalConfig2 = 0xD0;
      }
      reply(updateSettings(requested) ? CommandStatus::Ok
                                      : CommandStatus::ApplyFailed);
      return;
    }
    case CommandId::SetChannelMask: {
      if (!requireLength(1)) {
        return;
      }
      if (acquiring) {
        reply(CommandStatus::Busy);
        return;
      }
      ADS1299Settings requested = activeSettings;
      requested.channelEnabledMask = request.payload[0];
      reply(updateSettings(requested) ? CommandStatus::Ok
                                      : CommandStatus::ApplyFailed);
      return;
    }
    case CommandId::ResetStats:
      if (!requireLength(0)) {
        return;
      }
      resetAcquisitionStats();
      reply(CommandStatus::Ok);
      return;
    case CommandId::Ping:
      reply(CommandStatus::Ok, request.payload, request.payloadLength);
      return;
    default:
      reply(CommandStatus::InvalidCommand);
      return;
  }
}

void processWifiNativeBytes(const uint8_t *data, size_t length) {
  if (data == nullptr) {
    return;
  }
  for (size_t i = 0; i < length; ++i) {
    BCIKitProtocol::ControlRequest request{};
    const BCIKitProtocol::ParseResult result =
        wifiControlParser.feed(data[i], request);
    if (result == BCIKitProtocol::ParseResult::Complete) {
      // output="auto" remains unclaimed until a complete, CRC-valid native
      // COMMAND arrives. Claim before processing so even START as the first
      // command uses BCIKit framing.
      if (!bleTransport.sessionActive() && wifiTransport.claimNativeSession()) {
        processControlRequest(request, ControlTransport::WiFiTcp);
      }
    }
  }
}

void processBleNativeBytes(const uint8_t *data, size_t length) {
  if (data == nullptr) {
    return;
  }
  for (size_t i = 0; i < length; ++i) {
    BCIKitProtocol::ControlRequest request{};
    const BCIKitProtocol::ParseResult result =
        bleControlParser.feed(data[i], request);
    if (result != BCIKitProtocol::ParseResult::Complete) {
      continue;
    }
    if (wifiTransport.targetConfigured() || !bleTransport.claimSession()) {
      sendAck(ControlTransport::Ble, request.requestId, request.command,
              BCIKitProtocol::CommandStatus::Busy);
      continue;
    }
    processControlRequest(request, ControlTransport::Ble);
  }
}

void printOpenBCIStartup() {
  Serial.println("OpenBCI V3 8-16 channel");
  Serial.printf("ADS1299 Device ID: 0x%02X\n", ads1299.cachedId());
  Serial.println("Firmware: BCIKit v0.7.12");
  Serial.println("$$$");
}

bool reconfigureOpenBCI(const ADS1299Settings &requested) {
  const bool resume = acquiring;
  if (resume) {
    stopAcquisition();
  }
  const bool ok = updateSettings(requested);
  if (resume && ok) {
    startAcquisition();
  }
  return ok;
}

bool isOpenBCISingleCommand(char command) {
  return command == 'b' || command == 's' || command == 'v' ||
         command == 'V' || command == '?' || command == 'd' ||
         command == '0' || command == '-' || command == '=' ||
         command == '[' || command == ']' ||
         (command >= '1' && command <= '8') ||
         strchr("!@#$%^&*", command) != nullptr;
}

bool applyOpenBCISetting(char command) {
  ADS1299Settings requested = activeSettings;
  if (command == 'd') {
    requested.profile = ADS1299Profile::EegSrb2;
    requested.channelEnabledMask = 0xFF;
  } else if (command == '0') {
    requested.profile = ADS1299Profile::InputShort;
  } else if (command == '-' || command == '=' || command == '[' ||
             command == ']') {
    requested.profile = ADS1299Profile::InternalTest;
    requested.testSignalConfig2 = command == '-'   ? 0xD0
                                  : command == '=' ? 0xD1
                                  : command == '[' ? 0xD4
                                                   : 0xD5;
  } else if (command >= '1' && command <= '8') {
    requested.channelEnabledMask &=
        static_cast<uint8_t>(~(1u << (command - '1')));
  } else {
    const char *onCommands = "!@#$%^&*";
    const char *position = strchr(onCommands, command);
    if (position == nullptr) {
      return false;
    }
    requested.channelEnabledMask |=
        static_cast<uint8_t>(1u << (position - onCommands));
  }
  return reconfigureOpenBCI(requested);
}

void processOpenBCICommand(char command) {
  if (command == 's') {
    stopAcquisition();
    return;  // OpenBCI specifies no confirmation for s.
  }
  if (command == 'b') {
    if (acquiring) {
      return;
    }
    // A stray byte from the USB-UART bridge must not silently change the
    // default BCIKit mode and start an endless binary stream. Serial OpenBCI
    // hosts normally send 'v' first; humans can use `output openbci`.
    if (outputFormat != BCIKitProtocol::OutputFormat::OpenBCICyton) {
      Serial.println("ERR select OpenBCI first: send v or output openbci");
      return;
    }
    ADS1299Settings requested = activeSettings;
    requested.boardCount = 1;
    requested.sampleRateHz = 250;
    if (!updateSettings(requested)) {
      Serial.println("Failure: ADS1299 configuration failed$$$");
      return;
    }
    outputFormat = BCIKitProtocol::OutputFormat::OpenBCICyton;
    startAcquisition();
    return;  // OpenBCI specifies no confirmation for b.
  }
  if (acquiring && command != '0' && command != '-' && command != '=' &&
      command != '[' && command != ']' &&
      !(command >= '1' && command <= '8') &&
      strchr("!@#$%^&*", command) == nullptr) {
    return;
  }
  if (command == 'V') {
    Serial.println("v0.7.12$$$");
    return;
  }
  if (command == 'v') {
    stopAcquisition();
    ADS1299Settings defaults;
    defaults.boardCount = 1;
    defaults.sampleRateHz = 250;
    outputFormat = BCIKitProtocol::OutputFormat::OpenBCICyton;
    updateSettings(defaults);
    printOpenBCIStartup();
    return;
  }
  if (command == '?') {
    Serial.printf(
        "ADS1299: ID=0x%02X RATE=%u PROFILE=%s CHANNEL_MASK=0x%02X\n$$$\n",
        ads1299.cachedId(), activeSettings.sampleRateHz,
        ADS1299Driver::profileName(activeSettings.profile),
        activeSettings.channelEnabledMask);
    return;
  }

  const bool wasStreaming = acquiring;
  const bool ok = applyOpenBCISetting(command);
  if (!wasStreaming) {
    if (ok) {
      Serial.println("Success: OpenBCI setting applied$$$");
    } else {
      Serial.println("Failure: ADS1299 configuration failed$$$");
    }
  }
}

bool prepareWifiOutput(const String &requestedOutput) {
  if (bleTransport.sessionActive()) {
    return false;
  }
  if (acquiring) {
    stopAcquisition();
  }
  if (requestedOutput == "bcikit") {
    outputFormat = BCIKitProtocol::OutputFormat::BCIKitRawV1;
    return true;
  }
  if (requestedOutput != "raw") {
    return false;
  }
  ADS1299Settings requested = activeSettings;
  requested.boardCount = 1;
  if (!updateSettings(requested)) {
    return false;
  }
  outputFormat = BCIKitProtocol::OutputFormat::OpenBCICyton;
  return true;
}

bool startWifiStream() {
  return bleTransport.sessionActive() ? false : startAcquisition();
}

void stopWifiStream() { stopAcquisition(); }

String processWifiOpenBCICommand(const String &command) {
  if (command == "~~") {
    return "Sample rate is " + String(activeSettings.sampleRateHz) + "Hz$$$";
  }
  if (command.length() == 2 && command[0] == '~') {
    uint16_t rate = 0;
    if (command[1] == '6') {
      rate = 250;
    } else if (command[1] == '5') {
      rate = 500;
    } else if (command[1] == '4') {
      // BrainFlow's Cyton WiFi driver sends ~4 unconditionally during
      // prepare_session().  ADS1299 CONFIG1=0x94 is 1000 SPS with the
      // existing 2.048 MHz master clock, so handle it as a real rate rather
      // than acknowledging a rate the converter is not producing.
      rate = 1000;
    } else {
      return "Failure: unsupported OpenBCI WiFi sample rate$$$";
    }
    const bool resume = acquiring;
    if (resume) {
      stopAcquisition();
    }
    ADS1299Settings requested = activeSettings;
    requested.boardCount = 1;
    requested.sampleRateHz = rate;
    const bool ok = updateSettings(requested);
    outputFormat = BCIKitProtocol::OutputFormat::OpenBCICyton;
    if (resume && ok) {
      startAcquisition();
    }
    return ok ? "Sample rate set to " + String(rate) + "Hz$$$"
              : "Failure: ADS1299 configuration failed$$$";
  }
  if (command == "b") {
    return prepareWifiOutput("raw") && startAcquisition()
               ? "OK"
               : "Failure: unable to start stream$$$";
  }
  if (command == "s") {
    stopAcquisition();
    return "OK";
  }
  if (command == "V") {
    return "v0.7.12$$$";
  }
  if (command == "v") {
    stopAcquisition();
    ADS1299Settings defaults;
    defaults.boardCount = 1;
    defaults.sampleRateHz = 250;
    outputFormat = BCIKitProtocol::OutputFormat::OpenBCICyton;
    return updateSettings(defaults)
               ? "OpenBCI V3 8-16 channel\nFirmware: BCIKit v0.7.12\n$$$"
               : "Failure: ADS1299 configuration failed$$$";
  }
  if (command == "?") {
    return "ADS1299: ID=0x" + String(ads1299.cachedId(), HEX) +
           " RATE=" + String(activeSettings.sampleRateHz) + " PROFILE=" +
           ADS1299Driver::profileName(activeSettings.profile) +
           " CHANNEL_MASK=0x" + String(activeSettings.channelEnabledMask, HEX) +
           "\n$$$";
  }

  for (size_t i = 0; i < command.length(); ++i) {
    const char current = command[i];
    if (!isOpenBCISingleCommand(current) || current == 'b' || current == 's' ||
        current == 'v' || current == 'V' || current == '?') {
      return "Failure: unsupported OpenBCI command$$$";
    }
    if (!applyOpenBCISetting(current)) {
      return "Failure: ADS1299 configuration failed$$$";
    }
  }
  return "Success: OpenBCI setting applied$$$";
}

void processCommand(String command) {
  command.trim();
  command.toLowerCase();
  if (command.isEmpty()) {
    return;
  }

  // Human-readable commands are a bring-up interface. The professional host
  // should use binary commands, which remain framed while BCIKit is streaming.
  if (acquiring && command != "stop") {
    return;
  }

  if (command == "stop") {
    stopAcquisition();
    xSemaphoreTake(serialWriteMutex, portMAX_DELAY);
    Serial.println("OK stopped");
    xSemaphoreGive(serialWriteMutex);
  } else if (command == "start") {
    xSemaphoreTake(serialWriteMutex, portMAX_DELAY);
    if (startAcquisition()) {
      Serial.printf("OK streaming %s\n",
                    outputFormat == BCIKitProtocol::OutputFormat::BCIKitRawV1
                        ? "BCIKit Raw Stream v1"
                        : "OpenBCI Cyton 8ch");
    } else {
      Serial.println(
          "ERR start failed; BCIKit data requires WiFi, or select "
          "OpenBCI 250 SPS for UART");
    }
    xSemaphoreGive(serialWriteMutex);
  } else if (command == "info") {
    printInfo();
  } else if (command == "status") {
    printStatus();
  } else if (command == "help") {
    printHelp();
  } else if (command == "id") {
    Serial.printf("ADS1299 ID: 0x%02X (expected 0x3E)\n", ads1299.readId());
  } else if (command == "crc") {
    const uint8_t vector[] = "123456789";
    const uint16_t crc = BCIKitProtocol::crc16CcittFalse(vector, 9);
    Serial.printf("CRC test: 0x%04X (%s)\n", crc,
                  crc == 0x29B1 ? "PASS" : "FAIL");
  } else if (command.startsWith("rate ")) {
    const uint16_t rate = command.substring(5).toInt();
    if (rate != 250 && rate != 500) {
      Serial.println("ERR rate must be 250 or 500");
      return;
    }
    ADS1299Settings requested = activeSettings;
    requested.sampleRateHz = rate;
    Serial.println(updateSettings(requested) ? "OK rate applied"
                                             : "ERR register verify");
  } else if (command.startsWith("boards ")) {
    const int count = command.substring(7).toInt();
    if (count < 1 || count > 4 ||
        (outputFormat == BCIKitProtocol::OutputFormat::OpenBCICyton &&
         count != 1)) {
      Serial.println("ERR boards must be 1..4");
      return;
    }
    ADS1299Settings requested = activeSettings;
    requested.boardCount = static_cast<uint8_t>(count);
    Serial.println(updateSettings(requested) ? "OK board count applied"
                                             : "ERR register verify");
  } else if (command.startsWith("mode ")) {
    const String mode = command.substring(5);
    ADS1299Settings requested = activeSettings;
    if (mode == "eeg") {
      requested.profile = ADS1299Profile::EegSrb2;
    } else if (mode == "diff") {
      requested.profile = ADS1299Profile::Differential;
    } else if (mode == "test") {
      requested.profile = ADS1299Profile::InternalTest;
      requested.testSignalConfig2 = 0xD0;
    } else if (mode == "short") {
      requested.profile = ADS1299Profile::InputShort;
    } else {
      Serial.println("ERR mode must be eeg, diff, test, or short");
      return;
    }
    Serial.println(updateSettings(requested) ? "OK mode applied"
                                             : "ERR register verify");
  } else if (command.startsWith("channels ")) {
    const String value = command.substring(9);
    const long mask = strtol(value.c_str(), nullptr, 0);
    if (mask < 0 || mask > 0xFF) {
      Serial.println("ERR channels mask must be 0x00..0xFF");
      return;
    }
    ADS1299Settings requested = activeSettings;
    requested.channelEnabledMask = static_cast<uint8_t>(mask);
    Serial.println(updateSettings(requested) ? "OK channel mask applied"
                                             : "ERR register verify");
  } else if (command == "output bcikit") {
    outputFormat = BCIKitProtocol::OutputFormat::BCIKitRawV1;
    Serial.println("OK output BCIKit Raw Stream v1");
  } else if (command == "output openbci") {
    if (activeSettings.boardCount != 1) {
      Serial.println("ERR OpenBCI mode requires boards 1");
      return;
    }
    outputFormat = BCIKitProtocol::OutputFormat::OpenBCICyton;
    Serial.println("OK output OpenBCI Cyton 8ch");
  } else if (command == "wifi") {
    Serial.printf("ssid=%s ip=%s api=http://%s protocol=%u target=%s\n",
                  wifiTransport.name().c_str(),
                  wifiTransport.ipAddress().c_str(),
                  wifiTransport.ipAddress().c_str(),
                  static_cast<unsigned>(wifiTransport.protocol()),
                  wifiTransport.targetConnected() ? "connected" : "none");
  } else if (command == "resetstats") {
    resetAcquisitionStats();
    Serial.println("OK stats reset");
  } else {
    Serial.println("ERR unknown command; type help");
  }
}

}  // namespace

void setup() {
  Serial.begin(BCIKitConfig::SERIAL_BAUD);
  delay(800);
  Serial.println();
  Serial.println("BCIKit AFE8 ESP32-DevKitC firmware v0.7.12");

  drdyQueue = xQueueCreate(BCIKitConfig::DRDY_QUEUE_DEPTH, sizeof(uint64_t));
  sampleQueue = xQueueCreate(BCIKitConfig::SAMPLE_QUEUE_DEPTH,
                             sizeof(BCIKitProtocol::SampleRecord));
  serialWriteMutex = xSemaphoreCreateMutex();
  if (drdyQueue == nullptr || sampleQueue == nullptr ||
      serialWriteMutex == nullptr) {
    Serial.println("FATAL queue allocation failed");
    return;
  }

  if (!wifiTransport.begin(startWifiStream, stopWifiStream,
                           processWifiOpenBCICommand, prepareWifiOutput,
                           processWifiNativeBytes)) {
    Serial.println("WARN WiFi initialization failed; UART remains available");
  } else {
    Serial.printf("WiFi Direct ready: SSID=%s IP=%s HTTP=80\n",
                  wifiTransport.name().c_str(),
                  wifiTransport.ipAddress().c_str());
  }

  if (!bleTransport.begin(stopWifiStream, processBleNativeBytes)) {
    Serial.println("WARN BLE initialization failed; WiFi and UART remain available");
  } else {
    Serial.printf("BLE ready: name=%s service=BCIKit native protocol\n",
                  bleTransport.name().c_str());
  }

  if (!ads1299.begin(activeSettings)) {
    Serial.println("FATAL ADS1299 init failed (CLK, power, ID or register verify)");
    Serial.println("WiFi remains available for diagnostics at 192.168.4.1.");
    Serial.println("Check 2.048 MHz CLK, CS/RESET/PWDN, SPI Mode 1 and ID=0x3E.");
    return;
  }
  if (ads1299.cachedId() != 0x3E) {
    Serial.printf(
        "WARN ADS1299 ID readback unavailable (0x%02X); register profile "
        "verified. For multi-board daisy chains, presence is verified from "
        "each streaming status block.\n",
        ads1299.cachedId());
  }

  xTaskCreatePinnedToCore(acquisitionTask, "bcikit-acquire", 4096, nullptr, 20,
                          &acquisitionTaskHandle, 0);
  xTaskCreatePinnedToCore(transportTask, "bcikit-transport", 4096, nullptr, 5,
                          &transportTaskHandle, 1);

  // Ignore any line noise accumulated while the USB-UART bridge and WiFi were
  // starting. This prevents a stray byte from being mistaken for OpenBCI 'b'.
  while (Serial.available() > 0) {
    Serial.read();
  }

  Serial.println("READY. Type help, or use BCIKit binary control protocol v1.");
}

void loop() {
  bleTransport.service();
  wifiTransport.setExternalBusy(bleTransport.sessionActive());
  wifiTransport.service();
  while (Serial.available() > 0) {
    const uint8_t byte = static_cast<uint8_t>(Serial.read());
    if (serialControlParser.receiving() || byte == 0xA5) {
      if (!serialControlParser.receiving()) {
        commandLine = "";
      }
      BCIKitProtocol::ControlRequest request{};
      const BCIKitProtocol::ParseResult result =
          serialControlParser.feed(byte, request);
      if (result == BCIKitProtocol::ParseResult::Complete) {
        processControlRequest(request, ControlTransport::Serial);
      }
      continue;
    }

    const char c = static_cast<char>(byte);
    if (c == '\n' || c == '\r') {
      if (!commandLine.isEmpty()) {
        if (commandLine.length() == 1 &&
            isOpenBCISingleCommand(commandLine[0])) {
          processOpenBCICommand(commandLine[0]);
        } else {
          processCommand(commandLine);
        }
        commandLine = "";
      }
    } else if (commandLine.length() < 80) {
      commandLine += c;
      lastAsciiByteMs = millis();
    }
  }

  // OpenBCI commands are commonly sent as a single byte without a newline.
  if (commandLine.length() == 1 &&
      isOpenBCISingleCommand(commandLine[0]) &&
      millis() - lastAsciiByteMs >= 15) {
    processOpenBCICommand(commandLine[0]);
    commandLine = "";
  }
  delay(2);
}
