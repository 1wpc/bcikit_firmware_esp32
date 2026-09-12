#pragma once

#include <Arduino.h>
#include <WebServer.h>
#include <WiFi.h>
#include <WiFiUdp.h>

class BCIKitWiFi {
 public:
  using StartHandler = bool (*)();
  using StopHandler = void (*)();
  using CommandHandler = String (*)(const String &command);
  using OutputHandler = bool (*)(const String &output);
  using NativeReceiveHandler = void (*)(const uint8_t *data, size_t length);

  enum class Protocol : uint8_t { None = 0, Tcp = 1, Udp = 2 };
  enum class SessionProtocol : uint8_t {
    None = 0,
    Auto = 1,
    OpenBCI = 2,
    BCIKit = 3,
  };

  BCIKitWiFi();

  bool begin(StartHandler startHandler, StopHandler stopHandler,
             CommandHandler commandHandler, OutputHandler outputHandler,
             NativeReceiveHandler nativeReceiveHandler);
  void service();

  // Returns true when no network target is configured or the packet was sent.
  bool sendPacket(const uint8_t *data, size_t length);
  // Sends a BCIKit control response on the configured bidirectional TCP link.
  bool sendControlPacket(const uint8_t *data, size_t length);
  // Claims an output="auto" TCP target after a complete, CRC-valid BCIKit
  // COMMAND has been decoded. The caller must invoke this before processing
  // that command so START always uses the native output format.
  bool claimNativeSession();

  String name() const { return deviceName_; }
  String ipAddress() const;
  String sessionName() const;
  Protocol protocol() const { return static_cast<Protocol>(protocol_); }
  SessionProtocol sessionProtocol() const {
    return static_cast<SessionProtocol>(sessionProtocol_);
  }
  bool targetConfigured() const { return protocol_ != Protocol::None; }
  bool targetConnected();
  void setExternalBusy(bool busy) { externalBusy_ = busy; }
  uint32_t latencyUs() const { return latencyUs_; }

 private:
  WebServer server_;
  WiFiUDP dataUdp_;
  WiFiUDP ssdpUdp_;
  WiFiClient tcpClient_;
  SemaphoreHandle_t networkMutex_ = nullptr;

  StartHandler startHandler_ = nullptr;
  StopHandler stopHandler_ = nullptr;
  CommandHandler commandHandler_ = nullptr;
  OutputHandler outputHandler_ = nullptr;
  NativeReceiveHandler nativeReceiveHandler_ = nullptr;

  String deviceName_;
  String modelName_;
  String macAddress_;
  IPAddress targetIp_;
  uint16_t targetPort_ = 0;
  volatile Protocol protocol_ = Protocol::None;
  volatile SessionProtocol sessionProtocol_ = SessionProtocol::None;
  IPAddress sessionOwnerIp_;
  uint32_t pendingSinceMs_ = 0;
  volatile uint32_t lastActivityMs_ = 0;
  uint32_t stationLostSinceMs_ = 0;
  volatile uint8_t consecutiveSendFailures_ = 0;
  String output_ = "raw";
  bool delimiter_ = false;
  uint8_t redundancy_ = 1;
  uint32_t latencyUs_ = 10000;
  volatile bool externalBusy_ = false;

  void installRoutes();
  void handleRoot();
  void handleDescription();
  void handleAll();
  void handleBoard();
  void handleTcpGet();
  void handleTcpPost();
  void handleTcpDelete();
  void handleUdpPost();
  void handleUdpDelete();
  void handleStreamStart();
  void handleStreamStop();
  void handleCommand();
  void handleLatencyGet();
  void handleLatencyPost();
  void handleWifiDelete();
  void handleOptions();
  void handleNotFound();
  void serviceSsdp();
  void serviceSession();
  void serviceNativeTcpInput();

  void addCors();
  void sendText(int code, const String &body);
  void sendJson(int code, const String &body);
  String tcpInfoJson(bool connected) const;
  String macSuffix() const;
  bool configureTarget(Protocol protocol);
  bool claimSession(SessionProtocol protocol, const IPAddress &ownerIp);
  bool claimHttpSession(SessionProtocol protocol);
  bool sessionRequestAllowed(SessionProtocol protocol,
                             const IPAddress &requester) const;
  void markActivity();
  void clearTarget();

  static bool jsonString(const String &body, const char *key, String &value);
  static bool jsonInt(const String &body, const char *key, long &value);
  static bool jsonBool(const String &body, const char *key, bool &value);
};
