#include "OpenBCIProtocol.h"

#include <cstring>

namespace OpenBCIProtocol {

size_t encodeCytonPacket(const BCIKitProtocol::SampleRecord &sample,
                         uint8_t *output, size_t outputCapacity) {
  if (output == nullptr || outputCapacity < CYTON_PACKET_BYTES ||
      sample.boardCount != 1 || sample.payloadLength != 27) {
    return 0;
  }

  output[0] = 0xA0;
  output[1] = static_cast<uint8_t>(sample.sequence);
  // Skip the ADS1299 3-byte status word and preserve the 24 channel bytes in
  // their original signed 24-bit, MSB-first representation.
  memcpy(output + 2, sample.payload + 3, 24);
  memset(output + 26, 0, 6);  // No accelerometer on BCIKit AFE8.
  output[32] = 0xC0;          // Standard packet, accelerometer layout.
  return CYTON_PACKET_BYTES;
}

}  // namespace OpenBCIProtocol
