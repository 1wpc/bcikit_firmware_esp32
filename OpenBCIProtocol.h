#pragma once

#include <Arduino.h>

#include "BCIKitProtocol.h"

namespace OpenBCIProtocol {

constexpr size_t CYTON_PACKET_BYTES = 33;

// Encodes one physical ADS1299 board as the standard OpenBCI Cyton packet:
// A0 + sample_number + 8 * s24 channel data + 6 zero aux bytes + C0.
// Multi-board records are deliberately rejected because Cyton+Daisy uses a
// different alternating/averaging scheme and is not byte-for-byte equivalent.
size_t encodeCytonPacket(const BCIKitProtocol::SampleRecord &sample,
                         uint8_t *output, size_t outputCapacity);

}  // namespace OpenBCIProtocol
