#include "BCIKitBLE.h"

#include <NimBLEDevice.h>

namespace {

constexpr char kServiceUuid[] = "7d2ea28a-f7bd-485a-bd9d-92ad6ecfe93e";
constexpr char kControlRxUuid[] = "7d2ea28b-f7bd-485a-bd9d-92ad6ecfe93e";
constexpr char kControlTxUuid[] = "7d2ea28c-f7bd-485a-bd9d-92ad6ecfe93e";
constexpr char kSampleTxUuid[] = "7d2ea28d-f7bd-485a-bd9d-92ad6ecfe93e";
// macOS and recent mobile centrals can negotiate the full BLE ATT payload.
// Falling back to 247 remains valid; the sender queries the negotiated value.
constexpr uint16_t kPreferredMtu = 517;
// ATT MTU and Link-Layer data length are negotiated independently. A large
// ATT MTU without DLE fragments each notification into many short packets.
constexpr uint16_t kPreferredDataLength = 251;
// Apple centrals commonly reject a 7.5 ms peripheral request and retain their
// 30 ms default. Requesting the Apple-friendly 15-20 ms range instead lets
// CoreBluetooth select the low-latency interval needed by 16 channels.
constexpr uint16_t kStreamConnectionIntervalMin = 12;  // 15 ms
constexpr uint16_t kStreamConnectionIntervalMax = 16;  // 20 ms
constexpr uint16_t kStreamSupervisionTimeout = 400;    // 4 s
// A command response must be sent after outstanding sample notifications have
// drained from the controller.  Thirty milliseconds was enough for a light
// stream but not for the 16-channel / 250 SPS case on macOS, where the HCI
// notification queue can hold several connection events.
constexpr uint32_t kControlPriorityQuietMs = 250;

class ServerCallbacks final : public NimBLEServerCallbacks {
 public:
  explicit ServerCallbacks(BCIKitBLE &owner) : owner_(owner) {}

  void onConnect(NimBLEServer *, NimBLEConnInfo &info) override {
    owner_.onConnect(info.getConnHandle());
  }

  void onDisconnect(NimBLEServer *, NimBLEConnInfo &, int) override {
    owner_.onDisconnect();
  }

 private:
  BCIKitBLE &owner_;
};

class RxCallbacks final : public NimBLECharacteristicCallbacks {
 public:
  explicit RxCallbacks(BCIKitBLE &owner) : owner_(owner) {}

  void onWrite(NimBLECharacteristic *characteristic,
               NimBLEConnInfo &) override {
    const NimBLEAttValue value = characteristic->getValue();
    owner_.onWrite(value.data(), value.length());
  }

 private:
  BCIKitBLE &owner_;
};

}  // namespace

BCIKitBLE::BCIKitBLE() = default;

bool BCIKitBLE::begin(StopHandler stopHandler, ReceiveHandler receiveHandler) {
  stopHandler_ = stopHandler;
  receiveHandler_ = receiveHandler;
  rxQueue_ = xQueueCreate(8, sizeof(RxChunk));
  txMutex_ = xSemaphoreCreateMutex();
  if (rxQueue_ == nullptr || txMutex_ == nullptr) {
    return false;
  }

  const uint64_t mac = ESP.getEfuseMac();
  char suffix[5]{};
  snprintf(suffix, sizeof(suffix), "%04X", static_cast<unsigned>(mac & 0xFFFF));
  deviceName_ = "BCIKit-" + String(suffix);

  NimBLEDevice::init(deviceName_.c_str());
  NimBLEDevice::setMTU(kPreferredMtu);
  server_ = NimBLEDevice::createServer();
  if (server_ == nullptr) {
    return false;
  }
  server_->setCallbacks(new ServerCallbacks(*this));
  server_->advertiseOnDisconnect(true);

  NimBLEService *service = server_->createService(kServiceUuid);
  if (service == nullptr) {
    return false;
  }
  NimBLECharacteristic *controlRx = service->createCharacteristic(
      kControlRxUuid,
      NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
  controlTx_ = service->createCharacteristic(kControlTxUuid,
                                              NIMBLE_PROPERTY::NOTIFY);
  sampleTx_ = service->createCharacteristic(kSampleTxUuid,
                                             NIMBLE_PROPERTY::NOTIFY);
  if (controlRx == nullptr || controlTx_ == nullptr || sampleTx_ == nullptr) {
    return false;
  }
  controlRx->setCallbacks(new RxCallbacks(*this));
  NimBLEAdvertising *advertising = server_->getAdvertising();
  advertising->addServiceUUID(kServiceUuid);
  // A 128-bit service UUID leaves too little room for the full local name in
  // legacy advertising data. Put the name in the scan response so ordinary
  // active BLE scanners can reliably show BCIKit-XXXX.
  advertising->enableScanResponse(true);
  if (!advertising->setName(deviceName_.c_str())) {
    return false;
  }
  return advertising->start();
}

void BCIKitBLE::service() {
  // The first parameter update races the central's own MTU/DLE exchange on
  // macOS. Repeat it a few times after the link settles; this is a request,
  // so a central that declines simply keeps its existing interval.
  if (connected_ && server_ != nullptr && connectionTuneAttempts_ < 3 &&
      millis() - lastConnectionTuneMs_ >= 1000) {
    server_->updateConnParams(connectionId_, kStreamConnectionIntervalMin,
                              kStreamConnectionIntervalMax, 0,
                              kStreamSupervisionTimeout);
    lastConnectionTuneMs_ = millis();
    ++connectionTuneAttempts_;
  }
  if (disconnectPending_) {
    disconnectPending_ = false;
    if (stopHandler_ != nullptr) {
      stopHandler_();
    }
    if (rxQueue_ != nullptr) {
      xQueueReset(rxQueue_);
    }
  }

  RxChunk chunk{};
  while (rxQueue_ != nullptr && xQueueReceive(rxQueue_, &chunk, 0) == pdTRUE) {
    if (receiveHandler_ != nullptr && connected_) {
      receiveHandler_(chunk.data, chunk.length);
    }
  }
}

bool BCIKitBLE::claimSession() {
  if (!connected_) {
    return false;
  }
  sessionActive_ = true;
  return true;
}

bool BCIKitBLE::sendControlPacket(const uint8_t *data, size_t length) {
  // A control ACK must not be lost merely because the sample characteristic
  // briefly exhausts the controller's notification buffers.  It is also sent
  // from loop(), concurrently with the transport task, so serialize access to
  // NimBLE and retry the transient ENOMEM case.
  return connected_ && notify(controlTx_, data, length, 100, pdMS_TO_TICKS(1000));
}

bool BCIKitBLE::sendSamplePacket(const uint8_t *data, size_t length) {
  // Sampling itself is isolated in its own task.  Waiting for a few BLE
  // controller slots here applies back-pressure to the transport task instead
  // of discarding an entire batch whenever NimBLE has a short-lived ENOMEM.
  // The sample queue absorbs that delay and preserves ordering.
  // Keep the HCI queue clear when a peer has just written a command. Without
  // this reservation, high-rate sample notifications can starve a GET_STATUS
  // response long enough for desktop clients to tear down a healthy session.
  if (static_cast<int32_t>(millis() - controlPriorityUntilMs_) < 0) {
    return false;
  }
  return sessionActive() && notify(sampleTx_, data, length, 3, pdMS_TO_TICKS(4));
}

size_t BCIKitBLE::samplePacketCapacity() const {
  if (!connected_ || server_ == nullptr) {
    return 20;
  }
  const uint16_t peerMtu = server_->getPeerMTU(connectionId_);
  const size_t available = peerMtu > 3 ? peerMtu - 3 : 20;
  return available < 20 ? 20 : (available > 512 ? 512 : available);
}

uint16_t BCIKitBLE::peerMtu() const {
  return connected_ && server_ != nullptr ? server_->getPeerMTU(connectionId_)
                                           : 0;
}

uint16_t BCIKitBLE::connectionIntervalUnits() const {
  if (!connected_ || server_ == nullptr) {
    return 0;
  }
  return server_->getPeerInfoByHandle(connectionId_).getConnInterval();
}

void BCIKitBLE::onConnect(uint16_t connectionId) {
  connectionId_ = connectionId;
  connected_ = true;
  sessionActive_ = false;
  disconnectPending_ = false;
  controlPriorityUntilMs_ = 0;
  // Request a low-latency interval. service() repeats this after the link's
  // MTU/DLE exchange, when macOS is ready to evaluate it.
  server_->updateConnParams(connectionId_, kStreamConnectionIntervalMin,
                            kStreamConnectionIntervalMax, 0,
                            kStreamSupervisionTimeout);
  lastConnectionTuneMs_ = millis();
  connectionTuneAttempts_ = 1;
  // Request the maximum Bluetooth 4.2 Data Length Extension. Older centrals
  // safely keep their negotiated fallback.
  server_->setDataLen(connectionId_, kPreferredDataLength);
}

void BCIKitBLE::onDisconnect() {
  connected_ = false;
  sessionActive_ = false;
  controlPriorityUntilMs_ = 0;
  connectionTuneAttempts_ = 0;
  disconnectPending_ = true;
}

void BCIKitBLE::onWrite(const uint8_t *data, size_t length) {
  if (!connected_ || data == nullptr || length == 0 || rxQueue_ == nullptr) {
    return;
  }
  controlPriorityUntilMs_ = millis() + kControlPriorityQuietMs;
  while (length > 0) {
    RxChunk chunk{};
    chunk.length = static_cast<uint8_t>(
        length < sizeof(chunk.data) ? length : sizeof(chunk.data));
    memcpy(chunk.data, data, chunk.length);
    if (xQueueSend(rxQueue_, &chunk, 0) != pdTRUE) {
      return;
    }
    data += chunk.length;
    length -= chunk.length;
  }
}

bool BCIKitBLE::notify(NimBLECharacteristic *characteristic,
                       const uint8_t *data, size_t length, uint8_t attempts,
                       TickType_t lockWait) {
  if (!connected_ || characteristic == nullptr || data == nullptr ||
      length == 0 || server_ == nullptr || txMutex_ == nullptr ||
      attempts == 0 || xSemaphoreTake(txMutex_, lockWait) != pdTRUE) {
    return false;
  }
  bool sent = false;
  const size_t chunkSize = samplePacketCapacity();
  while (length > 0 && connected_) {
    const size_t current = length < chunkSize ? length : chunkSize;
    sent = false;
    for (uint8_t tryIndex = 0; tryIndex < attempts && connected_; ++tryIndex) {
      if (characteristic->notify(data, current, connectionId_)) {
        sent = true;
        break;
      }
      // Yield the HCI notification buffer to NimBLE before retrying. This is
      // intentionally only used by the reliable control path (attempts > 1).
      if (attempts > 1) {
        vTaskDelay(pdMS_TO_TICKS(2));
      }
    }
    if (!sent) {
      xSemaphoreGive(txMutex_);
      return false;
    }
    data += current;
    length -= current;
    taskYIELD();
  }
  xSemaphoreGive(txMutex_);
  return length == 0;
}
