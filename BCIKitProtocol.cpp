#include "BCIKitProtocol.h"

#include <cstring>

namespace {

void putU16Le(uint8_t *p, uint16_t value) {
  p[0] = static_cast<uint8_t>(value);
  p[1] = static_cast<uint8_t>(value >> 8);
}

void putU32Le(uint8_t *p, uint32_t value) {
  for (uint8_t i = 0; i < 4; ++i) {
    p[i] = static_cast<uint8_t>(value >> (8 * i));
  }
}

void putU64Le(uint8_t *p, uint64_t value) {
  for (uint8_t i = 0; i < 8; ++i) {
    p[i] = static_cast<uint8_t>(value >> (8 * i));
  }
}

uint16_t getU16Le(const uint8_t *p) {
  return static_cast<uint16_t>(p[0]) |
         static_cast<uint16_t>(p[1]) << 8;
}

uint32_t getU32Le(const uint8_t *p) {
  uint32_t value = 0;
  for (uint8_t i = 0; i < 4; ++i) {
    value |= static_cast<uint32_t>(p[i]) << (8 * i);
  }
  return value;
}

}  // namespace

namespace BCIKitProtocol {

uint16_t crc16CcittFalse(const uint8_t *data, size_t length) {
  uint16_t crc = 0xFFFF;
  while (length-- > 0) {
    crc ^= static_cast<uint16_t>(*data++) << 8;
    for (uint8_t i = 0; i < 8; ++i) {
      crc = (crc & 0x8000)
                ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                : static_cast<uint16_t>(crc << 1);
    }
  }
  return crc;
}

size_t encodeSample(const SampleRecord &sample, uint8_t *output,
                    size_t outputCapacity) {
  const size_t totalLength = 28 + sample.payloadLength;
  if (output == nullptr || sample.boardCount < 1 ||
      sample.boardCount > BCIKitConfig::MAX_BOARD_COUNT ||
      sample.payloadLength != sample.boardCount * BCIKitConfig::BYTES_PER_BOARD ||
      outputCapacity < totalLength) {
    return 0;
  }

  output[0] = 0xA5;
  output[1] = 0x5A;
  output[2] = PROTOCOL_VERSION;
  output[3] = static_cast<uint8_t>(FrameType::Sample);
  putU16Le(output + 4, sample.flags);
  putU32Le(output + 6, sample.sequence);
  putU64Le(output + 10, sample.timestampUs);
  putU16Le(output + 18, sample.sampleRateHz);
  output[20] = sample.boardCount;
  output[21] = static_cast<uint8_t>(sample.boardCount * 8);
  output[22] = sample.gainCode;
  output[23] = 0x00;
  putU16Le(output + 24, sample.payloadLength);
  memcpy(output + 26, sample.payload, sample.payloadLength);

  const uint16_t crc = crc16CcittFalse(output + 2, 24 + sample.payloadLength);
  putU16Le(output + 26 + sample.payloadLength, crc);
  return totalLength;
}

size_t encodeControlFrame(FrameType type, uint32_t requestId, uint8_t code,
                          CommandStatus status, const uint8_t *payload,
                          size_t payloadLength, uint8_t *output,
                          size_t outputCapacity) {
  const size_t totalLength = 14 + payloadLength;
  if (output == nullptr || outputCapacity < totalLength ||
      payloadLength > UINT16_MAX ||
      (payloadLength != 0 && payload == nullptr) || type == FrameType::Sample ||
      type == FrameType::Command) {
    return 0;
  }

  output[0] = 0xA5;
  output[1] = 0x5A;
  output[2] = PROTOCOL_VERSION;
  output[3] = static_cast<uint8_t>(type);
  putU32Le(output + 4, requestId);
  output[8] = code;
  output[9] = static_cast<uint8_t>(status);
  putU16Le(output + 10, static_cast<uint16_t>(payloadLength));
  if (payloadLength != 0) {
    memcpy(output + 12, payload, payloadLength);
  }
  const uint16_t crc = crc16CcittFalse(output + 2, 10 + payloadLength);
  putU16Le(output + 12 + payloadLength, crc);
  return totalLength;
}

size_t encodeDeviceInfo(uint32_t requestId, const DeviceInfo &info,
                        uint8_t *output, size_t outputCapacity) {
  uint8_t payload[24]{};
  payload[0] = info.firmwareMajor;
  payload[1] = info.firmwareMinor;
  payload[2] = info.firmwarePatch;
  payload[3] = PROTOCOL_VERSION;
  payload[4] = info.hardwareType;
  putU32Le(payload + 5, info.capabilities);
  payload[9] = info.maxBoardCount;
  payload[10] = info.sampleRateMask;
  payload[11] = info.outputFormatMask;
  putU32Le(payload + 12, info.serialBaud);
  putU64Le(payload + 16, info.deviceId);
  return encodeControlFrame(FrameType::DeviceInfo, requestId, 0,
                            CommandStatus::Ok, payload, sizeof(payload), output,
                            outputCapacity);
}

size_t encodeDeviceStatus(uint32_t requestId, const DeviceStatus &status,
                          uint8_t *output, size_t outputCapacity) {
  uint8_t payload[54]{};
  payload[0] = status.acquisitionState;
  payload[1] = static_cast<uint8_t>(status.outputFormat);
  payload[2] = status.profile;
  payload[3] = status.boardCount;
  putU16Le(payload + 4, status.sampleRateHz);
  payload[6] = status.gainCode;
  payload[7] = status.channelEnabledMask;
  payload[8] = status.registerVerified;
  payload[9] = status.adsId;
  putU32Le(payload + 10, status.nextSequence);
  putU64Le(payload + 14, status.uptimeUs);
  for (uint8_t i = 0; i < 8; ++i) {
    putU32Le(payload + 22 + i * 4, status.counters[i]);
  }
  return encodeControlFrame(FrameType::DeviceStatus, requestId, 0,
                            CommandStatus::Ok, payload, sizeof(payload), output,
                            outputCapacity);
}

ParseResult ControlParser::feed(uint8_t byte, ControlRequest &request) {
  if (index_ == 0) {
    if (byte == 0xA5) {
      buffer_[index_++] = byte;
    }
    return ParseResult::None;
  }
  if (index_ == 1) {
    if (byte == 0x5A) {
      buffer_[index_++] = byte;
    } else if (byte != 0xA5) {
      index_ = 0;
    }
    return ParseResult::None;
  }

  if (index_ >= sizeof(buffer_)) {
    index_ = 0;
    return ParseResult::Error;
  }
  buffer_[index_++] = byte;

  if (index_ == 12) {
    const uint16_t payloadLength = getU16Le(buffer_ + 10);
    if (payloadLength > MAX_COMMAND_PAYLOAD) {
      index_ = 0;
      return ParseResult::Error;
    }
  }
  if (index_ < 12) {
    return ParseResult::None;
  }

  const uint16_t payloadLength = getU16Le(buffer_ + 10);
  const size_t expectedLength = 14 + payloadLength;
  if (index_ < expectedLength) {
    return ParseResult::None;
  }

  const uint16_t expectedCrc = getU16Le(buffer_ + 12 + payloadLength);
  const uint16_t actualCrc = crc16CcittFalse(buffer_ + 2, 10 + payloadLength);
  const bool headerOk =
      buffer_[2] == PROTOCOL_VERSION &&
      buffer_[3] == static_cast<uint8_t>(FrameType::Command) && buffer_[9] == 0;
  if (!headerOk || actualCrc != expectedCrc) {
    index_ = 0;
    return ParseResult::Error;
  }

  request.requestId = getU32Le(buffer_ + 4);
  request.command = static_cast<CommandId>(buffer_[8]);
  request.payloadLength = payloadLength;
  if (payloadLength != 0) {
    memcpy(request.payload, buffer_ + 12, payloadLength);
  }
  index_ = 0;
  return ParseResult::Complete;
}

}  // namespace BCIKitProtocol
