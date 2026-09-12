#pragma once

#include <Arduino.h>

class NimBLECharacteristic;
class NimBLEServer;

class BCIKitBLE {
 public:
  using StopHandler = void (*)();
  using ReceiveHandler = void (*)(const uint8_t *data, size_t length);

  BCIKitBLE();

  bool begin(StopHandler stopHandler, ReceiveHandler receiveHandler);
  void service();
  bool claimSession();
  bool sendControlPacket(const uint8_t *data, size_t length);
  bool sendSamplePacket(const uint8_t *data, size_t length);
  size_t samplePacketCapacity() const;
  uint16_t peerMtu() const;
  uint16_t connectionIntervalUnits() const;

  bool connected() const { return connected_; }
  bool sessionActive() const { return connected_ && sessionActive_; }
  String name() const { return deviceName_; }

  // BLE stack callbacks enqueue work; ADS1299 and protocol work stays in the
  // Arduino loop/task context.
  void onConnect(uint16_t connectionId);
  void onDisconnect();
  void onWrite(const uint8_t *data, size_t length);

 private:
  struct RxChunk {
    uint8_t length;
    uint8_t data[64];
  };

  NimBLEServer *server_ = nullptr;
  NimBLECharacteristic *controlTx_ = nullptr;
  NimBLECharacteristic *sampleTx_ = nullptr;
  QueueHandle_t rxQueue_ = nullptr;
  StopHandler stopHandler_ = nullptr;
  ReceiveHandler receiveHandler_ = nullptr;
  String deviceName_;
  volatile bool connected_ = false;
  volatile bool sessionActive_ = false;
  volatile bool disconnectPending_ = false;
  volatile uint16_t connectionId_ = 0;
  volatile uint32_t lastConnectionTuneMs_ = 0;
  volatile uint8_t connectionTuneAttempts_ = 0;
  // A host command must always be able to receive its ACK/status reply even
  // while the sample characteristic is saturated. onWrite arms this brief
  // quiet window before the command is dispatched from loop().
  volatile uint32_t controlPriorityUntilMs_ = 0;

  bool notify(NimBLECharacteristic *characteristic, const uint8_t *data,
              size_t length, uint8_t attempts, TickType_t lockWait);
  SemaphoreHandle_t txMutex_ = nullptr;
};
