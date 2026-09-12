#pragma once

#include <Arduino.h>

#include "ADS1299Driver.h"
#include "BCIKitConfig.h"

namespace BCIKitProtocol {

constexpr uint8_t PROTOCOL_VERSION = 0x01;

enum class FrameType : uint8_t {
  Sample = 0x01,
  DeviceInfo = 0x02,
  DeviceStatus = 0x03,
  Ack = 0x04,
  Event = 0x05,
  Command = 0x80,
};

enum class OutputFormat : uint8_t {
  BCIKitRawV1 = 0,
  OpenBCICyton = 1,
};

enum class CommandId : uint8_t {
  GetInfo = 0x01,
  GetStatus = 0x02,
  Start = 0x03,
  Stop = 0x04,
  SetOutputFormat = 0x05,
  SetSampleRate = 0x06,
  SetBoardCount = 0x07,
  SetProfile = 0x08,
  SetChannelMask = 0x09,
  ResetStats = 0x0A,
  Ping = 0x0B,
};

enum class CommandStatus : uint8_t {
  Ok = 0,
  InvalidCommand = 1,
  InvalidLength = 2,
  InvalidValue = 3,
  Busy = 4,
  ApplyFailed = 5,
  Unsupported = 6,
  CrcError = 7,
};

constexpr uint32_t CAP_BCIKIT_RAW_V1 = 1u << 0;
constexpr uint32_t CAP_OPENBCI_CYTON = 1u << 1;
constexpr uint32_t CAP_MULTI_BOARD = 1u << 2;
constexpr uint32_t CAP_DRDY_TIMESTAMP = 1u << 3;
constexpr uint32_t CAP_CRC16 = 1u << 4;
constexpr uint32_t CAP_INTERNAL_TEST = 1u << 5;
constexpr uint32_t CAP_INPUT_SHORT = 1u << 6;
constexpr uint32_t CAP_CHANNEL_MASK = 1u << 7;
constexpr uint32_t CAP_BINARY_CONTROL = 1u << 8;
constexpr uint32_t CAP_WIFI_OPENBCI_API = 1u << 9;
constexpr uint32_t CAP_WIFI_BCIKIT_STREAM = 1u << 10;
constexpr uint32_t CAP_WIFI_BCIKIT_CONTROL = 1u << 11;
constexpr uint32_t CAP_AUTO_PROTOCOL_SESSION = 1u << 12;
constexpr uint32_t CAP_BLE_BCIKIT_STREAM = 1u << 13;
constexpr uint32_t CAP_BLE_BCIKIT_CONTROL = 1u << 14;

constexpr uint16_t FLAG_TIME_VALID = 1u << 0;
constexpr uint16_t FLAG_DROPPED_BEFORE = 1u << 1;
constexpr uint16_t FLAG_REGISTER_VERIFIED = 1u << 2;
constexpr uint16_t FLAG_TEST_SIGNAL = 1u << 3;
constexpr uint16_t FLAG_INPUT_SHORT = 1u << 4;

struct SampleRecord {
  uint32_t sequence;
  uint64_t timestampUs;
  uint16_t flags;
  uint16_t sampleRateHz;
  uint8_t boardCount;
  uint8_t gainCode;
  uint16_t payloadLength;
  uint8_t payload[BCIKitConfig::MAX_ADS_FRAME_BYTES];
};

struct DeviceInfo {
  uint8_t firmwareMajor;
  uint8_t firmwareMinor;
  uint8_t firmwarePatch;
  uint8_t hardwareType;
  uint32_t capabilities;
  uint8_t maxBoardCount;
  uint8_t sampleRateMask;
  uint8_t outputFormatMask;
  uint32_t serialBaud;
  uint64_t deviceId;
};

struct DeviceStatus {
  uint8_t acquisitionState;
  OutputFormat outputFormat;
  uint8_t profile;
  uint8_t boardCount;
  uint16_t sampleRateHz;
  uint8_t gainCode;
  uint8_t channelEnabledMask;
  uint8_t registerVerified;
  uint8_t adsId;
  uint32_t nextSequence;
  uint64_t uptimeUs;
  uint32_t counters[8];
};

constexpr size_t MAX_COMMAND_PAYLOAD = 32;

struct ControlRequest {
  uint32_t requestId;
  CommandId command;
  uint16_t payloadLength;
  uint8_t payload[MAX_COMMAND_PAYLOAD];
};

enum class ParseResult : uint8_t { None, Complete, Error };

class ControlParser {
 public:
  ParseResult feed(uint8_t byte, ControlRequest &request);
  bool receiving() const { return index_ != 0; }
  void reset() { index_ = 0; }

 private:
  uint8_t buffer_[14 + MAX_COMMAND_PAYLOAD]{};
  size_t index_ = 0;
};

uint16_t crc16CcittFalse(const uint8_t *data, size_t length);
size_t encodeSample(const SampleRecord &sample, uint8_t *output,
                    size_t outputCapacity);
size_t encodeControlFrame(FrameType type, uint32_t requestId, uint8_t code,
                          CommandStatus status, const uint8_t *payload,
                          size_t payloadLength, uint8_t *output,
                          size_t outputCapacity);
size_t encodeDeviceInfo(uint32_t requestId, const DeviceInfo &info,
                        uint8_t *output, size_t outputCapacity);
size_t encodeDeviceStatus(uint32_t requestId, const DeviceStatus &status,
                          uint8_t *output, size_t outputCapacity);

}  // namespace BCIKitProtocol
