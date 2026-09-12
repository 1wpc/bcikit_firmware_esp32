#pragma once

#include <Arduino.h>

// BCIKit ESP32-DevKitC V4 (classic ESP32) Host Adapter pin map.
// GPIO6..GPIO11 are connected to the module's SPI flash and must not be used.
// The mapping also avoids UART0 and the boot-strapping pins 0, 2, 5, 12 and 15.
namespace BCIKitConfig {

constexpr int PIN_ADS_CLK = 25;    // AFE J3-11, LEDC 2.048 MHz master clock
constexpr int PIN_ADS_DRDY = 34;   // AFE J3-12, input-only GPIO is ideal here
constexpr int PIN_ADS_START = 26;  // AFE J3-8
constexpr int PIN_ADS_RESET = 27;  // AFE J3-9, active low
constexpr int PIN_ADS_PWDN = 32;   // AFE J3-10, active low
constexpr int PIN_ADS_CS = 22;     // AFE J3-7, active low
constexpr int PIN_ADS_MOSI = 23;   // AFE J3-6, VSPI MOSI -> ADS1299 DIN
constexpr int PIN_ADS_SCK = 18;    // AFE J3-5, VSPI SCLK
constexpr int PIN_ADS_MISO = 19;   // AFE J3-14, VSPI MISO <- chain return

constexpr uint32_t ADS_MASTER_CLOCK_HZ = 2048000;
constexpr uint32_t ADS_SPI_CLOCK_HZ = 4000000;

constexpr uint8_t DEFAULT_BOARD_COUNT = 1;
constexpr uint16_t DEFAULT_SAMPLE_RATE_HZ = 250;
constexpr uint32_t SERIAL_BAUD = 115200;

constexpr size_t MAX_BOARD_COUNT = 4;
constexpr size_t BYTES_PER_BOARD = 27;
constexpr size_t MAX_ADS_FRAME_BYTES = MAX_BOARD_COUNT * BYTES_PER_BOARD;
constexpr size_t MAX_PROTOCOL_FRAME_BYTES = 28 + MAX_ADS_FRAME_BYTES;

constexpr size_t DRDY_QUEUE_DEPTH = 16;
// BLE transport may briefly wait for controller credits.  At 250 SPS this
// holds more than a quarter second of complete frames without touching the
// DRDY/SPI acquisition path.
constexpr size_t SAMPLE_QUEUE_DEPTH = 80;
constexpr uint8_t INITIAL_FRAMES_TO_DISCARD = 3;

// OpenBCI WiFi Shield compatible WiFi Direct defaults. The SSID suffix is
// generated from the last two bytes of the ESP32 SoftAP MAC address.
constexpr uint8_t WIFI_AP_IP_0 = 192;
constexpr uint8_t WIFI_AP_IP_1 = 168;
constexpr uint8_t WIFI_AP_IP_2 = 4;
constexpr uint8_t WIFI_AP_IP_3 = 1;
constexpr uint16_t WIFI_HTTP_PORT = 80;
constexpr uint16_t WIFI_SSDP_PORT = 1900;
constexpr uint32_t WIFI_DEFAULT_LATENCY_US = 10000;

}  // namespace BCIKitConfig
