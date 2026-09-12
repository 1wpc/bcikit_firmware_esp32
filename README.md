# BCIKit AFE8 ESP32-DevKitC V4 Arduino 固件 v0.7.12

这是 BCIKit AFE8 新板面向经典 ESP32-DevKitC V4 的参考固件，可直接使用 Arduino IDE 打开和编译。

本目录可独立作为公开仓库使用，许可证为 [MIT](LICENSE)。仓库只包含 ESP32 主控适配层和 AFE8 固件，不包含 BCIKit 私有上位机或其他未公开平台代码。

引脚依据：[乐鑫 ESP32-DevKitC V4 官方用户指南](https://docs.espressif.com/projects/esp-dev-kits/zh_CN/latest/esp32/esp32-devkitc/user_guide.html)。

实现内容：

- LEDC 连续输出 ADS1299 所需的 2.048 MHz 主时钟。
- SPI Mode 1、4 MHz；`RDATAC` 连续采样期间 `CS` 保持拉低，命令和停止采样时按 ADS1299 时序切换。
- ADS1299 硬件复位、ID `0x3E` 检查、寄存器写入和掩码读回验证。
- BCIKit 原生模式支持 250/500 SPS；OpenBCI Cyton WiFi 兼容模式支持 BrainFlow 初始化所需的 1000 SPS。
- EEG SRB2、全差分、内部测试、输入短接四种配置。
- 1 到 4 块 AFE8 菊花链，保留 `27 × board_count` 原始 wire order。
- DRDY ISR 只记录时间戳并通知高优先级采集任务。
- FreeRTOS 采集队列与 UART 发送任务分离。
- BCIKit Raw Stream Protocol v1、32 位序号、64 位时间戳、CRC-16/CCITT-FALSE。
- 可切换的 OpenBCI Cyton 8 通道、33 字节数据输出。
- OpenBCI WiFi Direct：`OpenBCI-XXXX`、`192.168.4.1`、SSDP 自动发现、HTTP API、TCP/UDP/UDPx3。
- WiFi 同时支持 OpenBCI `raw` 和 BCIKit 原生二进制数据帧。
- BCIKit WiFi TCP 双向控制：同一连接接收 COMMAND，并返回 ACK、DEVICE_INFO、DEVICE_STATUS 和 SAMPLE。
- BCIKit BLE 双向控制：独立控制与采样 characteristic，适合自研桌面端和移动端。
- WiFi 自动协议会话：OpenBCI `raw` 请求或 BCIKit 有效 COMMAND 自动认领会话，后来的冲突协议返回 `409 BUSY`。
- `output="auto"` 待识别模式与 5 秒握手超时；TCP 断开后自动停止采集并释放会话。
- BCIKit 二进制设备控制协议：能力发现、请求 ID、ACK、配置和错误统计。
- 状态头、DRDY 超时、队列溢出和传输丢弃统计。

## 1. Arduino IDE

1. 安装 Arduino IDE 2.x。
2. Boards Manager 安装 `esp32 by Espressif Systems`，本工程已用 `3.3.7` 编译验证。
3. 在 Library Manager 安装 `NimBLE-Arduino`（已用 `2.5.1` 编译验证）；这是 BLE 所需依赖。
4. 用 Arduino IDE 打开 `esp32_devkitc_ads1299.ino`。
5. Board 选择 `ESP32 Dev Module`。
6. CPU Frequency 选择 `240MHz`，Flash Size/Partition Scheme 按实际模组选择。
7. 上传后打开串口，波特率设为 `115200`，行尾选 `Newline`。

ESP32-DevKitC V4 使用板载 USB-UART 桥接芯片，并没有 ESP32-S3 那样的原生 USB CDC。v0.7.12 将 UART0 固定为 115200，主要用于日志、控制和 OpenBCI 8 通道 250 SPS 兼容输出；BCIKit 原生数据和双向机器控制通过 WiFi TCP 或 BLE 传输。BLE 流式传输在控制命令到达时会暂时让出通知队列，并对短暂的控制器拥塞进行有限重试，避免状态命令被数据流饿死；同时请求 251 字节 BLE 数据长度扩展。双板通知最多合并两帧，避免 macOS 控制器信用不足时大 ATT 通知阻塞发送任务。

## 2. ESP32-DevKitC V4 引脚

该分配避开了 GPIO6..11 SPI Flash 管脚、GPIO1/3 下载串口、GPIO0/2/5/12/15 启动绑带脚，也不依赖 WROVER 可能占用的 GPIO16/17。ADS1299 SPI 使用 ESP32 的 VSPI 控制器。

| DevKitC GPIO | DevKitC 排针 | AFE8 J3 | 信号 |
| ---: | --- | ---: | --- |
| GPIO25 | J2 pin 9 / IO25 | 11 | CLK 2.048 MHz |
| GPIO34 | J2 pin 5 / IO34 | 12 | DRDY_CHAIN |
| GPIO26 | J2 pin 10 / IO26 | 8 | START |
| GPIO27 | J2 pin 11 / IO27 | 9 | RESET |
| GPIO32 | J2 pin 7 / IO32 | 10 | PWDN |
| GPIO22 | J3 pin 3 / IO22 | 7 | CS |
| GPIO23 | J3 pin 2 / IO23 | 6 | DIN / MOSI |
| GPIO18 | J3 pin 9 / IO18 | 5 | SCLK |
| GPIO19 | J3 pin 8 / IO19 | 14 | CHAIN_MISO_RETURN / MISO |
| GND | J2 pin 14 或 J3 pin 1/7 | 1/3/4/16 | GND |

AFE8 的 J3 pin 2 还需要 5 V。ESP32-DevKitC 与 AFE8 必须共地，所有数字 GPIO 都是 3.3 V，禁止用 5 V GPIO 直连。无人体连接的台架测试可以从 DevKitC 5V 脚取电；连接人体时必须改用电池或合规隔离供电，并断开 USB、示波器地等非隔离路径。

建议在主控驱动源附近给 `CLK` 和 `SCLK` 各串联 33 Ohm，实际值用示波器检查振铃后调整。

## 3. 第一次上电

先不要接人体电极。确认板卡拨码处于单板角色：`DAISY_IN -> GND`、唯一 `DRDY -> DRDY_CHAIN`、本板 `DOUT -> CHAIN_MISO_RETURN`。

串口控制台执行：

```text
crc
id
status
mode test
```

预期：

- `crc` 返回 `0x29B1 PASS`。
- `id` 返回 `0x3E`。
- 示波器在 CLK 上看到约 2.048 MHz、接近 50% 占空比。
- `mode test` 后，DRDY 为 250 Hz。
- 内部测试信号约 0.9765625 Hz，gain 24 时理想幅度约 83886 counts。

随后连接 `OpenBCI-XXXX`，使用 `bcikit_wifi_smoke.py --output auto` 启动并验证自动识别和完整数据流。若要单独验证 UART OpenBCI 兼容流，可执行：

```text
mode test
output openbci
start
```

此时 115200 UART 输出标准 33 字节 OpenBCI 包；发送 `s` 或 `stop` 停止。二进制流中其他文本命令会被静默忽略，避免回复破坏数据帧。

## 4. 命令

```text
help
info
status
id
crc
output bcikit
output openbci
rate 250
rate 500
boards 1
boards 2
boards 3
boards 4
mode eeg
mode diff
mode test
mode short
channels 0xff
resetstats
start
stop
```

`mode eeg` 是默认 SRB2 公共参考、gain 24 配置。改变输出格式、采样率、板数、模式或通道掩码前必须先停止采集。

文本命令只用于人工 Bring-up。正式上位机应使用 [BCIKit Device Control Protocol v1](../../../protocol/BCIKit_Device_Control_Protocol_v1.md)，它提供带 CRC 和 `request_id` 的命令、ACK、设备能力和状态帧。Python 行为参考实现位于 [`sdk/python/bcikit_protocol.py`](../../../sdk/python/bcikit_protocol.py)。

## 5. OpenBCI 兼容模式

OpenBCI Cyton 输出固定要求单板、8 通道；UART 建议使用传统 250 SPS。WiFi 支持 250、500 和 1000 SPS，其中当前 BrainFlow Cyton WiFi 驱动在初始化时会固定选择 1000 SPS：

```text
stop
boards 1
rate 250
output openbci
start
```

每个 packet 为：

```text
A0 + sample_number + 24 channel bytes + 6 zero aux bytes + C0
```

也可以直接使用 OpenBCI 单字节命令：

| 命令 | 行为 |
| --- | --- |
| `b` | 已处于 OpenBCI 模式时开始采集；先发送 `v` 或执行 `output openbci` |
| `s` | 停止采集 |
| `v` | 软复位到 OpenBCI 默认配置并打印 `$$$` 结束标记 |
| `V` | 返回固件版本 |
| `?` | 返回 ADS1299 配置摘要 |
| `1`..`8` | 关闭对应通道 |
| `! @ # $ % ^ & *` | 打开对应通道 |
| `d` | 恢复默认 EEG 通道配置 |
| `0` | 输入短接模式 |
| `-` / `=` | 1倍幅度慢速/快速内部测试信号 |
| `[` / `]` | 2倍幅度慢速/快速内部测试信号 |

当前承诺的是 Cyton 8 通道数据包和上述常用控制命令。尚未实现 `x...X` 的逐通道增益/MUX完整命令、`z...Z` lead-off 命令、SD卡命令和 Cyton+Daisy 的交替平均格式。多板数据必须使用 BCIKit Raw Stream v1，避免丢失状态字、时间戳和精确同步信息。

## 6. OpenBCI WiFi Direct

上电后 ESP32 同时建立一个与官方 WiFi Shield 默认值一致的开放热点：

| 参数 | 值 |
| --- | --- |
| SSID | `OpenBCI-XXXX`，后四位来自 SoftAP MAC |
| 密码 | 无 |
| 设备 IP | `192.168.4.1` |
| HTTP API | TCP 80 |
| 自动发现 | SSDP `239.255.255.250:1900` |
| 默认 latency 字段 | `10000 us` |
| 默认 OpenBCI 数据 | Cyton 33-byte raw packet |

电脑连接 `OpenBCI-XXXX` 后，可以先打开 `http://192.168.4.1`。OpenBCI GUI 中选择 Cyton、WiFi Shield；自动发现失败时手工输入 `192.168.4.1`。v0.6.1 保留 BrainFlow 初始化所需的 `~4`（1000 SPS）命令；ADS1299 使用现有 2.048 MHz 主时钟和 `CONFIG1=0x94` 真实工作在 1000 SPS，不是伪造应答。

已实现的官方兼容接口：

```text
GET  /all
GET  /board
GET  /tcp
POST /tcp
DELETE /tcp
POST /udp
DELETE /udp
POST /command
GET  /stream/start
GET  /stream/stop
GET/POST /latency
GET  /version
GET  /description.xml
```

官方模式示例请求：

```json
{"ip":"192.168.4.2","port":12345,"output":"raw","delimiter":false,"latency":10000}
```

ESP32 会主动连接电脑监听的 TCP 端口，然后把标准 OpenBCI raw packet 发给电脑。UDP 请求还接受 `"redundancy":true`，此时每个数据报发送三次，兼容 GUI 的 UDPx3 选项。

一旦 `/tcp` 或 `/udp` 配置成功，采样数据只发送到 WiFi，UART 保持为日志和控制终端，不再同步输出二进制数据。删除 TCP/UDP 目标时固件会先停止采集，避免数据突然回落到 UART。未配置网络目标时，仅 OpenBCI 单板 250 SPS 可以通过 UART 输出。

115200 UART 无法承载单板 BCIKit 原生帧的最低数据量（约 13.75 kB/s），所以未配置 WiFi 时，固件只允许 OpenBCI 单板 250 SPS 走 UART。BCIKit 模式执行 `start` 会明确报错，不会再产生阻塞式乱码流。

自研上位机使用 TCP 时，推荐把扩展字段设为 `auto`：

```json
{"ip":"192.168.4.2","port":12345,"output":"auto","delimiter":false,"latency":10000}
```

反向 TCP 建立后，上位机首先发送任意一条 CRC 正确的 BCIKit COMMAND（推荐 `PING`）。固件只在完整 COMMAND 校验通过后认领 BCIKit 会话，因此 TCP 分包、启动噪声或无效 CRC 不会触发协议切换。5 秒内没有完成握手时，待识别连接会被关闭并释放。

也可显式指定 BCIKit，兼容 v0.5.0 上位机：

```json
{"ip":"192.168.4.2","port":12345,"output":"bcikit","delimiter":false,"latency":10000}
```

此时网络中传输完整的 BCIKit Raw Stream v1 帧。详细接口见 [OpenBCI WiFi 兼容说明](../../../protocol/OpenBCI_WiFi_Compatibility.md)，电脑端测试见 [`bcikit_wifi_smoke.py`](../../../sdk/python/bcikit_wifi_smoke.py)。

v0.5.0 起，该 TCP 连接同时接收 BCIKit COMMAND；v0.6.0 增加自动认领与首会话锁定。v0.6.1 修正多板菊花链不能使用单板 ID 作为连接门槛的问题。推荐握手为：

```text
PING -> GET_INFO -> STOP -> SET_OUTPUT_FORMAT(0)
-> SET_BOARD_COUNT -> SET_SAMPLE_RATE -> SET_PROFILE
-> SET_CHANNEL_MASK -> GET_STATUS -> START
```

ACK、DEVICE_INFO、DEVICE_STATUS 和 SAMPLE 都从同一 TCP 字节流返回，必须按 `frame_type` 分发，并使用 `request_id` 匹配命令响应。DEVICE_INFO capabilities bit 11 表示支持此双向控制能力。UDP 仍然只用于单向 SAMPLE。

OpenBCI GUI 继续发送 `output="raw"`，因此无需修改；自研软件发送 `output="auto"` 后以有效 BCIKit COMMAND 完成识别。一个协议会话存在时，来自冲突协议或不同监听端口的 `/tcp`、`/udp`、`/command` 或 `/stream/start` 返回 HTTP `409 BUSY`；完全相同的 POST 重试保持幂等。`DELETE /tcp`、`DELETE /udp` 或 TCP 实际断开会停止采集并释放所有权。`GET /all` 和 `GET /tcp` 的 `session` 字段为 `none`、`auto`、`openbci` 或 `bcikit`。

正式自研上位机请从 [BCIKit 上位机协议文档入口](../../../protocol/README.md) 开始，并以 [上位机集成指南](../../../protocol/BCIKit_Host_Integration_Guide_v1.md) 和 [BCIKit WiFi Transport Profile v1](../../../protocol/BCIKit_WiFi_Transport_v1.md) 为实现合同，不需要依赖 OpenBCI 数据格式。

## 7. BCIKit BLE

上电后，设备同时广播名称为 `BCIKit-XXXX` 的 BLE 外设（后四位来自芯片 MAC）。BLE 仅服务 BCIKit 原生协议，不模拟 OpenBCI Cyton 蓝牙接口；OpenBCI GUI 仍使用本固件的 WiFi Direct 兼容路径。

BLE 和 WiFi 的原生会话互斥。BLE 端订阅通知并成功发送首个 CRC 正确的 BCIKit COMMAND 后取得设备会话；此时 WiFi 的建链、命令和启动请求返回 HTTP `409 BUSY`。反之，只要 WiFi 已配置数据目标，BLE 命令会收到 `ACK(STATUS_BUSY)`。BLE 断开时固件自动 STOP、清空待处理命令并释放会话。

BLE V1 为保证稳定性，正式支持范围为：单板 250/500 SPS，双板 250 SPS。三、四板及双板 500 SPS 的 `START` 会返回失败；如需更高吞吐量，请使用 WiFi TCP。完整 UUID、分包和上位机接入顺序见 [BCIKit BLE Transport Profile v1](../../../protocol/BCIKit_BLE_Transport_v1.md)。

## 8. 多板规则

- `boards N` 必须与真实板数一致，不自动探测。单板配置使用逐寄存器 RREG 校验；多板共享链路不把不可靠的 RREG 当作成功门槛，而是在后续正式 `START` 后检查每块 ADS1299 的 27 字节状态头。连续三帧链路异常会立即停采并撤销 `register_verified`，因此错误板数或坏链路不会静默输出。
- 从多板切回单板时，固件会短暂进入可直接读回的 CONFIG1 模式，校验全部配置并读取 ADS1299 ID；只有读到 `0x3E` 才确认切换成功，随后恢复单板采集配置，避免把链路读回的 `0x00` 误报成芯片异常。
- ESP32 启动、板数变化以及初始化校验重试时，会通过共享 `PWDN` 对整条 AFE 链做受控掉电复位，清除拨码或返回路径变化前残留的串行链状态；拨码仍必须在断电后调整。
- 多板配置按 BCIKit 板间回传拓扑设置 `CONFIG1`，并通过连续帧长度及每块 27 字节状态头做端到端校验；不能只根据通用 ADS1299 拓扑假定寄存器位，也不能把共享链路上的逐寄存器 RREG 当作多板成功门槛。
- Host -> A -> B -> C 时，payload 原始顺序为 C、B、A。
- 固件不重排 payload，所以协议 `PHYSICAL_ORDER` 标志保持 0。
- 共享 `CS/DIN` 的 daisy-chain 不支持逐板寄存器 multiple readback；`ads_id`
  只作为单板诊断。多板在线状态以每个 `27` 字节块的 `0xC` 状态头为准。
- 全链只能有一个 DRDY 返回驱动源，建议首板。
- 全链只能有一个 DOUT 返回驱动源，必须是末板。
- 4 板 500 SPS 约 68 kB/s，115200 baud 不够。

## 9. 当前版本边界

- WiFi 在 ADS1299 初始化之前启动；即使 AFE 初始化失败，仍应看到 `OpenBCI-XXXX` 并可访问 `192.168.4.1` 进行诊断。
- 当前版本实现官方 WiFi Direct 兼容路径、BCIKit TCP 原生双向控制和 BCIKit BLE；尚未实现路由器 Station 配网、网页 captive portal、JSON 纳伏输出、MQTT、BLE 配对/加密、配置持久化和 OTA。
- 连续 3 帧状态头错误时固件会安全停止，检查 `board_count`、SPI、拨码和线缆后重新执行 `start`。
- v0.6.1 会在热点客户端离开、TCP 明确断开或连续发送失败时自动停止采集并释放会话；同一客户端使用新监听端口重连时会安全替换旧目标。因此直接切换 WiFi、应用崩溃或异常中断不要求 `DELETE /tcp` 必须成功送达。正常暂停采集不会因固定空闲时间而丢失TCP目标。
- `ledcReadFreq()` 只证明 ESP32 外设配置成功；最终必须用示波器验证 AFE8 J3 pin 11 上的实际频率和波形。
- 多板共享 CS/DIN 时，寄存器读回只能证明返回链路上的读值，生产测试仍应先逐块单板验证。
- 本设备用于教学、科研和开发，不是医疗诊断设备；人体连接时应使用电池或合规隔离方案，调试器、USB、示波器地线不得形成危险的人体接地路径。
