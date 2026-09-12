#include "BCIKitWiFi.h"

#include <ESPmDNS.h>

#include "BCIKitConfig.h"

namespace {

constexpr char kWifiShieldVersion[] = "v2.0.5-bcikit.4";
constexpr uint32_t kAutoHandshakeTimeoutMs = 5000;
constexpr uint32_t kStationLossGraceMs = 1000;
constexpr uint8_t kSendFailureLimit = 3;
const IPAddress kSsdpMulticast(239, 255, 255, 250);

String twoDigitHex(uint8_t value) {
  char text[3];
  snprintf(text, sizeof(text), "%02X", value);
  return String(text);
}

String jsonKey(const char *key) {
  return String('"') + key + '"';
}

size_t skipSpace(const String &text, size_t position) {
  while (position < text.length() && isspace(text[position])) {
    ++position;
  }
  return position;
}

}  // namespace

BCIKitWiFi::BCIKitWiFi() : server_(BCIKitConfig::WIFI_HTTP_PORT) {}

bool BCIKitWiFi::begin(StartHandler startHandler, StopHandler stopHandler,
                       CommandHandler commandHandler,
                       OutputHandler outputHandler,
                       NativeReceiveHandler nativeReceiveHandler) {
  startHandler_ = startHandler;
  stopHandler_ = stopHandler;
  commandHandler_ = commandHandler;
  outputHandler_ = outputHandler;
  nativeReceiveHandler_ = nativeReceiveHandler;
  latencyUs_ = BCIKitConfig::WIFI_DEFAULT_LATENCY_US;
  networkMutex_ = xSemaphoreCreateMutex();
  if (networkMutex_ == nullptr) {
    return false;
  }

  WiFi.mode(WIFI_AP);
  const IPAddress ip(BCIKitConfig::WIFI_AP_IP_0, BCIKitConfig::WIFI_AP_IP_1,
                     BCIKitConfig::WIFI_AP_IP_2, BCIKitConfig::WIFI_AP_IP_3);
  const IPAddress subnet(255, 255, 255, 0);
  if (!WiFi.softAPConfig(ip, ip, subnet)) {
    return false;
  }

  uint8_t mac[6]{};
  WiFi.softAPmacAddress(mac);
  macAddress_ = twoDigitHex(mac[0]) + ":" + twoDigitHex(mac[1]) + ":" +
                twoDigitHex(mac[2]) + ":" + twoDigitHex(mac[3]) + ":" +
                twoDigitHex(mac[4]) + ":" + twoDigitHex(mac[5]);
  deviceName_ = "OpenBCI-" + twoDigitHex(mac[4]) + twoDigitHex(mac[5]);
  modelName_ = "PTW-0001-" + twoDigitHex(mac[4]) + twoDigitHex(mac[5]);

  // The official shield's WiFi Direct network is open and named OpenBCI-XXXX.
  if (!WiFi.softAP(deviceName_.c_str())) {
    return false;
  }

  installRoutes();
  server_.begin();
  MDNS.begin(deviceName_.c_str());
  MDNS.addService("http", "tcp", BCIKitConfig::WIFI_HTTP_PORT);
  ssdpUdp_.beginMulticast(kSsdpMulticast, BCIKitConfig::WIFI_SSDP_PORT);
  return true;
}

void BCIKitWiFi::service() {
  server_.handleClient();
  serviceSsdp();
  serviceSession();
  serviceNativeTcpInput();
}

String BCIKitWiFi::ipAddress() const { return WiFi.softAPIP().toString(); }

String BCIKitWiFi::sessionName() const {
  switch (sessionProtocol_) {
    case SessionProtocol::Auto:
      return "auto";
    case SessionProtocol::OpenBCI:
      return "openbci";
    case SessionProtocol::BCIKit:
      return "bcikit";
    default:
      return "none";
  }
}

bool BCIKitWiFi::targetConnected() {
  if (protocol_ == Protocol::Tcp) {
    return tcpClient_.connected();
  }
  return protocol_ == Protocol::Udp && targetPort_ != 0;
}

bool BCIKitWiFi::sendPacket(const uint8_t *data, size_t length) {
  if (protocol_ == Protocol::None || data == nullptr || length == 0) {
    return true;
  }
  if (xSemaphoreTake(networkMutex_, pdMS_TO_TICKS(10)) != pdTRUE) {
    return false;
  }

  bool ok = true;
  if (protocol_ == Protocol::Tcp) {
    if (!tcpClient_.connected()) {
      ok = false;
    } else {
      ok = tcpClient_.write(data, length) == length;
      if (ok && delimiter_) {
        static const uint8_t delimiter[] = {'\r', '\n'};
        ok = tcpClient_.write(delimiter, sizeof(delimiter)) ==
             sizeof(delimiter);
      }
    }
  } else {
    for (uint8_t copy = 0; copy < redundancy_; ++copy) {
      if (!dataUdp_.beginPacket(targetIp_, targetPort_) ||
          dataUdp_.write(data, length) != length) {
        ok = false;
      }
      if (delimiter_ && dataUdp_.write(reinterpret_cast<const uint8_t *>("\r\n"),
                                       2) != 2) {
        ok = false;
      }
      if (!dataUdp_.endPacket()) {
        ok = false;
      }
    }
  }
  xSemaphoreGive(networkMutex_);
  if (protocol_ == Protocol::Tcp) {
    if (ok) {
      markActivity();
    } else if (consecutiveSendFailures_ < UINT8_MAX) {
      consecutiveSendFailures_ =
          static_cast<uint8_t>(consecutiveSendFailures_ + 1);
    }
  }
  return ok;
}

bool BCIKitWiFi::sendControlPacket(const uint8_t *data, size_t length) {
  if (data == nullptr || length == 0 || protocol_ != Protocol::Tcp ||
      output_ != "bcikit" || networkMutex_ == nullptr ||
      xSemaphoreTake(networkMutex_, pdMS_TO_TICKS(100)) != pdTRUE) {
    return false;
  }
  const bool ok = protocol_ == Protocol::Tcp && output_ == "bcikit" &&
                  tcpClient_.connected() &&
                  tcpClient_.write(data, length) == length;
  xSemaphoreGive(networkMutex_);
  if (ok) {
    markActivity();
  } else if (consecutiveSendFailures_ < UINT8_MAX) {
    consecutiveSendFailures_ =
        static_cast<uint8_t>(consecutiveSendFailures_ + 1);
  }
  return ok;
}

bool BCIKitWiFi::claimNativeSession() {
  if (protocol_ != Protocol::Tcp ||
      (output_ != "auto" && output_ != "bcikit")) {
    return false;
  }
  return claimSession(SessionProtocol::BCIKit, sessionOwnerIp_);
}

void BCIKitWiFi::serviceSession() {
  const bool sessionActive = targetConfigured() ||
                             sessionProtocol_ != SessionProtocol::None;
  if (sessionActive && WiFi.softAPgetStationNum() == 0) {
    if (stationLostSinceMs_ == 0) {
      stationLostSinceMs_ = millis();
    } else if (millis() - stationLostSinceMs_ >= kStationLossGraceMs) {
      if (stopHandler_ != nullptr) {
        stopHandler_();
      }
      clearTarget();
      return;
    }
  } else {
    stationLostSinceMs_ = 0;
  }

  if (consecutiveSendFailures_ >= kSendFailureLimit) {
    if (stopHandler_ != nullptr) {
      stopHandler_();
    }
    clearTarget();
    return;
  }

  if (protocol_ == Protocol::Tcp && targetPort_ != 0 &&
      !tcpClient_.connected()) {
    if (stopHandler_ != nullptr) {
      stopHandler_();
    }
    clearTarget();
    return;
  }

  const bool waitingForHandshake =
      sessionProtocol_ == SessionProtocol::Auto || !targetConfigured();
  if (waitingForHandshake && pendingSinceMs_ != 0 &&
      millis() - pendingSinceMs_ >= kAutoHandshakeTimeoutMs) {
    if (stopHandler_ != nullptr) {
      stopHandler_();
    }
    clearTarget();
    return;
  }

}

void BCIKitWiFi::serviceNativeTcpInput() {
  if (nativeReceiveHandler_ == nullptr || protocol_ != Protocol::Tcp ||
      (output_ != "auto" && output_ != "bcikit") ||
      networkMutex_ == nullptr) {
    return;
  }

  uint8_t input[64];
  size_t length = 0;
  if (xSemaphoreTake(networkMutex_, pdMS_TO_TICKS(10)) != pdTRUE) {
    return;
  }
  if (protocol_ == Protocol::Tcp &&
      (output_ == "auto" || output_ == "bcikit") &&
      tcpClient_.connected()) {
    const int available = tcpClient_.available();
    const size_t requested =
        available > 0 ? min(static_cast<size_t>(available), sizeof(input)) : 0;
    if (requested > 0) {
      const int received = tcpClient_.read(input, requested);
      if (received > 0) {
        length = static_cast<size_t>(received);
        markActivity();
      }
    }
  }
  xSemaphoreGive(networkMutex_);

  // Invoke the parser after releasing the network mutex. A complete command
  // may synchronously send its ACK back through sendControlPacket().
  if (length > 0) {
    nativeReceiveHandler_(input, length);
  }
}

void BCIKitWiFi::installRoutes() {
  server_.on("/", HTTP_GET, [this]() { handleRoot(); });
  server_.on("/description.xml", HTTP_GET,
             [this]() { handleDescription(); });
  server_.on("/all", HTTP_GET, [this]() { handleAll(); });
  server_.on("/board", HTTP_GET, [this]() { handleBoard(); });
  server_.on("/tcp", HTTP_GET, [this]() { handleTcpGet(); });
  server_.on("/tcp", HTTP_POST, [this]() { handleTcpPost(); });
  server_.on("/tcp", HTTP_DELETE, [this]() { handleTcpDelete(); });
  server_.on("/udp", HTTP_POST, [this]() { handleUdpPost(); });
  server_.on("/udp", HTTP_DELETE, [this]() { handleUdpDelete(); });
  server_.on("/stream/start", HTTP_GET,
             [this]() { handleStreamStart(); });
  server_.on("/stream/stop", HTTP_GET,
             [this]() { handleStreamStop(); });
  server_.on("/command", HTTP_POST, [this]() { handleCommand(); });
  server_.on("/latency", HTTP_GET, [this]() { handleLatencyGet(); });
  server_.on("/latency", HTTP_POST, [this]() { handleLatencyPost(); });
  server_.on("/version", HTTP_GET,
             [this]() { sendText(200, kWifiShieldVersion); });
  server_.on("/wifi", HTTP_DELETE, [this]() { handleWifiDelete(); });
  server_.on("/wifi/delete", HTTP_GET, [this]() { handleWifiDelete(); });

  const char *optionsRoutes[] = {
      "/",          "/all",        "/board",       "/tcp",
      "/udp",       "/stream/start", "/stream/stop", "/command",
      "/latency",   "/version",    "/wifi",        "/wifi/delete",
  };
  for (const char *route : optionsRoutes) {
    server_.on(route, HTTP_OPTIONS, [this]() { handleOptions(); });
  }
  server_.onNotFound([this]() { handleNotFound(); });
}

void BCIKitWiFi::handleRoot() {
  const String html =
      "<!doctype html><html><meta name='viewport' content='width=device-width'>"
      "<title>BCIKit OpenBCI WiFi</title><h1>BCIKit WiFi</h1><p>Device: " +
      deviceName_ + "</p><p>IP: " + ipAddress() +
      "</p><p>OpenBCI WiFi Shield compatible API is ready.</p></html>";
  addCors();
  server_.send(200, "text/html", html);
}

void BCIKitWiFi::handleDescription() {
  const String uuid = "uuid:" + deviceName_;
  const String xml =
      "<?xml version=\"1.0\"?><root xmlns=\"urn:schemas-upnp-org:device-1-0\">"
      "<specVersion><major>1</major><minor>0</minor></specVersion><device>"
      "<deviceType>urn:schemas-upnp-org:device:Basic:1</deviceType>"
      "<friendlyName>PTW - OpenBCI Wifi Shield</friendlyName>"
      "<manufacturer>Push The World LLC</manufacturer>"
      "<manufacturerURL>https://openbci.com</manufacturerURL>"
      "<modelDescription>BCIKit OpenBCI WiFi compatibility layer</modelDescription>"
      "<modelName>" +
      modelName_ +
      "</modelName><modelNumber>929000226503</modelNumber><serialNumber>" +
      deviceName_ + "</serialNumber><UDN>" + uuid +
      "</UDN></device></root>";
  addCors();
  server_.send(200, "text/xml", xml);
}

void BCIKitWiFi::handleAll() {
  const String json =
      "{\"board_connected\":true,\"heap\":" + String(ESP.getFreeHeap()) +
      ",\"ip\":\"" + ipAddress() + "\",\"mac\":\"" + macAddress_ +
      "\",\"name\":\"" + deviceName_ +
      "\",\"num_channels\":8,\"version\":\"" + kWifiShieldVersion +
      "\",\"session\":\"" + sessionName() +
      "\",\"latency\":" + String(latencyUs_) + "}";
  sendJson(200, json);
}

void BCIKitWiFi::handleBoard() {
  sendJson(200,
           "{\"board_connected\":true,\"board_type\":\"cyton\","
           "\"num_channels\":8,\"gains\":[24,24,24,24,24,24,24,24]}");
}

void BCIKitWiFi::handleTcpGet() {
  sendJson(200, tcpInfoJson(tcpClient_.connected()));
}

void BCIKitWiFi::handleTcpPost() {
  if (configureTarget(Protocol::Tcp)) {
    sendJson(200, tcpInfoJson(true));
  }
}

void BCIKitWiFi::handleTcpDelete() {
  if (stopHandler_ != nullptr) {
    stopHandler_();
  }
  clearTarget();
  sendJson(200, tcpInfoJson(false));
}

void BCIKitWiFi::handleUdpPost() {
  if (configureTarget(Protocol::Udp)) {
    sendJson(200, tcpInfoJson(true));
  }
}

void BCIKitWiFi::handleUdpDelete() {
  if (stopHandler_ != nullptr) {
    stopHandler_();
  }
  clearTarget();
  sendJson(200, tcpInfoJson(false));
}

void BCIKitWiFi::handleStreamStart() {
  if (!claimHttpSession(SessionProtocol::OpenBCI)) {
    return;
  }
  if (startHandler_ != nullptr && startHandler_()) {
    markActivity();
    sendText(200, "OK");
  } else {
    sendText(500, "Error: unable to start stream");
  }
}

void BCIKitWiFi::handleStreamStop() {
  if (stopHandler_ != nullptr) {
    stopHandler_();
  }
  markActivity();
  sendText(200, "OK");
}

void BCIKitWiFi::handleCommand() {
  String command;
  if (!jsonString(server_.arg("plain"), "command", command)) {
    sendText(403, "command");
    return;
  }
  if (command.length() > 31) {
    sendText(501, "Error: Sent more than 31 chars");
    return;
  }
  if (!claimHttpSession(SessionProtocol::OpenBCI)) {
    return;
  }
  markActivity();
  const String response = commandHandler_ == nullptr
                              ? String("Error: command handler unavailable")
                              : commandHandler_(command);
  sendText(response.startsWith("Failure:") ? 400 : 200, response);
}

void BCIKitWiFi::handleLatencyGet() {
  sendText(200, String(latencyUs_));
}

void BCIKitWiFi::handleLatencyPost() {
  long latency = 0;
  if (!jsonInt(server_.arg("plain"), "latency", latency) || latency < 50 ||
      latency > 1000000) {
    sendText(403, "latency");
    return;
  }
  latencyUs_ = static_cast<uint32_t>(latency);
  sendText(200, "OK");
}

void BCIKitWiFi::handleWifiDelete() {
  if (stopHandler_ != nullptr) {
    stopHandler_();
  }
  clearTarget();
  sendText(200, "WiFi Direct mode active; no station credentials stored");
}

void BCIKitWiFi::handleOptions() {
  addCors();
  server_.send(200, "text/plain", "");
}

void BCIKitWiFi::handleNotFound() { sendText(404, "Route Not Found"); }

void BCIKitWiFi::serviceSsdp() {
  const int packetSize = ssdpUdp_.parsePacket();
  if (packetSize <= 0) {
    return;
  }
  char request[512]{};
  const int length = ssdpUdp_.read(request, sizeof(request) - 1);
  if (length <= 0) {
    return;
  }
  request[length] = '\0';
  const String message(request);
  if (message.indexOf("M-SEARCH") < 0 ||
      (message.indexOf("urn:schemas-upnp-org:device:Basic:1") < 0 &&
       message.indexOf("ssdp:all") < 0)) {
    return;
  }

  const String response =
      "HTTP/1.1 200 OK\r\nCACHE-CONTROL: max-age=120\r\nEXT:\r\nLOCATION: http://" +
      ipAddress() +
      "/description.xml\r\nSERVER: Arduino/3.3 UPnP/1.1 " + modelName_ +
      "/2.0.5\r\nST: urn:schemas-upnp-org:device:Basic:1\r\nUSN: uuid:" +
      deviceName_ +
      "::urn:schemas-upnp-org:device:Basic:1\r\n\r\n";
  ssdpUdp_.beginPacket(ssdpUdp_.remoteIP(), ssdpUdp_.remotePort());
  ssdpUdp_.write(reinterpret_cast<const uint8_t *>(response.c_str()),
                 response.length());
  ssdpUdp_.endPacket();
}

void BCIKitWiFi::addCors() {
  server_.sendHeader("Access-Control-Allow-Origin", "*");
  server_.sendHeader("Access-Control-Allow-Methods", "POST,DELETE,GET,OPTIONS");
  server_.sendHeader("Access-Control-Allow-Headers", "Content-Type");
}

void BCIKitWiFi::sendText(int code, const String &body) {
  addCors();
  server_.send(code, "text/plain", body + "\r\n");
}

void BCIKitWiFi::sendJson(int code, const String &body) {
  addCors();
  server_.send(code, "application/json", body);
}

String BCIKitWiFi::tcpInfoJson(bool connected) const {
  return "{\"connected\":" + String(connected ? "true" : "false") +
         ",\"delimiter\":" + String(delimiter_ ? "true" : "false") +
         ",\"ip\":\"" + targetIp_.toString() + "\",\"output\":\"" +
         output_ + "\",\"session\":\"" + sessionName() +
         "\",\"port\":" + String(targetPort_) +
         ",\"latency\":" + String(latencyUs_) + "}";
}

String BCIKitWiFi::macSuffix() const {
  return deviceName_.substring(deviceName_.length() - 4);
}

bool BCIKitWiFi::configureTarget(Protocol protocol) {
  serviceSession();
  if (externalBusy_) {
    sendText(409, "BUSY: BLE session owns the device");
    return false;
  }
  const String body = server_.arg("plain");
  String ipText;
  long port = 0;
  if (!jsonString(body, "ip", ipText)) {
    sendText(403, "ip");
    return false;
  }
  IPAddress parsedIp;
  if (!parsedIp.fromString(ipText)) {
    sendText(505, "Error: unable to parse ip address");
    return false;
  }
  if (!jsonInt(body, "port", port) || port < 1 || port > 65535) {
    sendText(403, "port");
    return false;
  }

  String requestedOutput = "raw";
  jsonString(body, "output", requestedOutput);
  requestedOutput.toLowerCase();
  if (requestedOutput != "raw" && requestedOutput != "bcikit" &&
      requestedOutput != "auto") {
    sendText(506, "Error: 'output' must be raw, bcikit, or auto");
    return false;
  }
  if (protocol == Protocol::Udp && requestedOutput == "auto") {
    sendText(506, "Error: output auto requires TCP");
    return false;
  }

  const SessionProtocol requestedSession =
      requestedOutput == "raw"
          ? SessionProtocol::OpenBCI
          : requestedOutput == "bcikit" ? SessionProtocol::BCIKit
                                         : SessionProtocol::Auto;
  const IPAddress requester = server_.client().remoteIP();
  // Treat an exact retry as idempotent. A new listener port from the same
  // station is an explicit reconnect/takeover: this covers application
  // crashes and WiFi switching where DELETE /tcp could not be delivered.
  // Different station IPs still cannot take over an active session.
  if (targetConfigured()) {
    const bool sameTarget = targetIp_ == parsedIp &&
                            targetPort_ == static_cast<uint16_t>(port) &&
                            protocol_ == protocol;
    const bool sameOutput = requestedOutput == output_ ||
                            (requestedOutput == "auto" &&
                             sessionProtocol_ == SessionProtocol::BCIKit);
    if (sameTarget && sameOutput && targetConnected()) {
      markActivity();
      return true;
    }
    if (sessionOwnerIp_ != requester) {
      sendText(409, "BUSY: another protocol session owns the device");
      return false;
    }
    if (stopHandler_ != nullptr) {
      stopHandler_();
    }
    clearTarget();
  }
  if (!sessionRequestAllowed(requestedSession, requester)) {
    sendText(409, "BUSY: another protocol session owns the device");
    return false;
  }
  if (requestedSession != SessionProtocol::Auto &&
      !claimSession(requestedSession, requester)) {
    sendText(506, "Error: unable to apply requested output mode");
    return false;
  }

  bool boolValue = false;
  if (jsonBool(body, "delimiter", boolValue)) {
    delimiter_ = boolValue;
  }
  long latency = 0;
  if (jsonInt(body, "latency", latency) && latency >= 50 &&
      latency <= 1000000) {
    latencyUs_ = static_cast<uint32_t>(latency);
  }
  redundancy_ = 1;
  if (jsonBool(body, "redundancy", boolValue) && boolValue) {
    redundancy_ = 3;
  }

  if (xSemaphoreTake(networkMutex_, pdMS_TO_TICKS(500)) != pdTRUE) {
    sendText(503, "Error: network busy");
    return false;
  }
  tcpClient_.stop();
  targetIp_ = parsedIp;
  targetPort_ = static_cast<uint16_t>(port);
  output_ = requestedOutput;
  protocol_ = protocol;
  bool connected = true;
  if (protocol == Protocol::Tcp) {
    connected = tcpClient_.connect(targetIp_, targetPort_);
    if (connected) {
      tcpClient_.setNoDelay(true);
    } else {
      protocol_ = Protocol::None;
    }
  } else {
    connected = dataUdp_.begin(0) != 0;
    if (!connected) {
      protocol_ = Protocol::None;
    }
  }
  xSemaphoreGive(networkMutex_);

  if (!connected) {
    clearTarget();
    sendJson(504, tcpInfoJson(false));
    return false;
  }
  if (requestedSession == SessionProtocol::Auto) {
    sessionProtocol_ = SessionProtocol::Auto;
    sessionOwnerIp_ = requester;
    pendingSinceMs_ = millis();
  } else {
    pendingSinceMs_ = 0;
  }
  markActivity();
  return true;
}

bool BCIKitWiFi::sessionRequestAllowed(
    SessionProtocol protocol, const IPAddress &requester) const {
  if (sessionProtocol_ == SessionProtocol::None) {
    return true;
  }
  if (sessionOwnerIp_ != requester) {
    return false;
  }
  return sessionProtocol_ == protocol ||
         sessionProtocol_ == SessionProtocol::Auto ||
         (protocol == SessionProtocol::Auto &&
          sessionProtocol_ == SessionProtocol::BCIKit);
}

bool BCIKitWiFi::claimSession(SessionProtocol protocol,
                              const IPAddress &ownerIp) {
  if (protocol != SessionProtocol::OpenBCI &&
      protocol != SessionProtocol::BCIKit) {
    return false;
  }
  if (sessionProtocol_ == protocol && sessionOwnerIp_ == ownerIp) {
    return true;
  }
  if (sessionProtocol_ != SessionProtocol::None &&
      sessionProtocol_ != SessionProtocol::Auto) {
    return false;
  }
  if (sessionProtocol_ == SessionProtocol::Auto &&
      sessionOwnerIp_ != ownerIp) {
    return false;
  }

  const String selectedOutput =
      protocol == SessionProtocol::BCIKit ? "bcikit" : "raw";
  if (outputHandler_ != nullptr && !outputHandler_(selectedOutput)) {
    return false;
  }
  output_ = selectedOutput;
  sessionProtocol_ = protocol;
  sessionOwnerIp_ = ownerIp;
  pendingSinceMs_ = 0;
  return true;
}

bool BCIKitWiFi::claimHttpSession(SessionProtocol protocol) {
  serviceSession();
  if (externalBusy_) {
    sendText(409, "BUSY: BLE session owns the device");
    return false;
  }
  if (claimSession(protocol, server_.client().remoteIP())) {
    if (!targetConfigured()) {
      pendingSinceMs_ = millis();
    }
    markActivity();
    return true;
  }
  sendText(409, "BUSY: another protocol session owns the device");
  return false;
}

void BCIKitWiFi::markActivity() {
  lastActivityMs_ = millis();
  consecutiveSendFailures_ = 0;
}

void BCIKitWiFi::clearTarget() {
  if (networkMutex_ != nullptr &&
      xSemaphoreTake(networkMutex_, pdMS_TO_TICKS(500)) == pdTRUE) {
    tcpClient_.stop();
    dataUdp_.stop();
    protocol_ = Protocol::None;
    targetPort_ = 0;
    targetIp_ = IPAddress();
    output_ = "raw";
    sessionProtocol_ = SessionProtocol::None;
    sessionOwnerIp_ = IPAddress();
    pendingSinceMs_ = 0;
    lastActivityMs_ = 0;
    stationLostSinceMs_ = 0;
    consecutiveSendFailures_ = 0;
    xSemaphoreGive(networkMutex_);
  }
}

bool BCIKitWiFi::jsonString(const String &body, const char *key,
                            String &value) {
  size_t position = body.indexOf(jsonKey(key));
  if (position == static_cast<size_t>(-1)) {
    return false;
  }
  position = body.indexOf(':', position);
  if (position == static_cast<size_t>(-1)) {
    return false;
  }
  position = skipSpace(body, position + 1);
  if (position >= body.length() || body[position] != '"') {
    return false;
  }
  const size_t end = body.indexOf('"', position + 1);
  if (end == static_cast<size_t>(-1)) {
    return false;
  }
  value = body.substring(position + 1, end);
  return true;
}

bool BCIKitWiFi::jsonInt(const String &body, const char *key, long &value) {
  size_t position = body.indexOf(jsonKey(key));
  if (position == static_cast<size_t>(-1)) {
    return false;
  }
  position = body.indexOf(':', position);
  if (position == static_cast<size_t>(-1)) {
    return false;
  }
  position = skipSpace(body, position + 1);
  char *end = nullptr;
  value = strtol(body.c_str() + position, &end, 10);
  return end != body.c_str() + position;
}

bool BCIKitWiFi::jsonBool(const String &body, const char *key, bool &value) {
  size_t position = body.indexOf(jsonKey(key));
  if (position == static_cast<size_t>(-1)) {
    return false;
  }
  position = body.indexOf(':', position);
  if (position == static_cast<size_t>(-1)) {
    return false;
  }
  position = skipSpace(body, position + 1);
  if (body.substring(position).startsWith("true")) {
    value = true;
    return true;
  }
  if (body.substring(position).startsWith("false")) {
    value = false;
    return true;
  }
  return false;
}
