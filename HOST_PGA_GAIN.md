# 上位机对接：PGA 增益（固件 v0.7.16）

通过 BCIKit 原生控制连接（WiFi TCP 或 BLE）发送 `SET_GAIN`。此设置是整个设备的统一增益：所有板上启用的通道使用同一档。暂不支持逐通道设置。ESP32 重新上电后恢复默认 24 倍。

| 增益 | 1 | 2 | 4 | 6 | 8 | 12 | 24 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `gain_code` | 0 | 1 | 2 | 3 | 4 | 5 | 6 |

连接流程：

1. `GET_INFO (0x01)`，检查 `capabilities` bit15 (`0x00008000`)。没有该位的旧固件维持 24 倍，不发送新命令。
2. `STOP (0x04)`；等待 ACK。
3. 按实际板数发送 `SET_BOARD_COUNT (0x07)`，随后发送 `SET_GAIN (0x0C)`，payload 恰好 1 字节，内容为上表中的 code。继续设置采样率、profile 和通道掩码时，增益保持不变。
4. 收到同 `request_id` 的 `ACK (0x04)` 且 status=0 后，发送 `GET_STATUS (0x02)`，确认 payload offset 6 的 `gain_code` 与请求相同，offset 8 的 `register_verified` 为 1。
5. `START (0x03)` 后，每个 BCIKit SAMPLE offset 22 也包含 `gain_code`。用帧内 code 换算幅值：`µV = raw_count × 4.5 / gain / (2^23 − 1) × 10^6`。

Control frame 编码：`A5 5A 01 80` + `request_id` (u32 little endian) + `0C 00 01 00` + `gain_code` + CRC16 (u16 little endian)。CRC-16/CCITT-FALSE 从版本字节 `01` 计算到 payload 最后一个字节，初值 `FFFF`、多项式 `1021`。

错误状态：采集中返回 4 (`BUSY`)；payload 长度不是 1 返回 2 (`INVALID_LENGTH`)；code 不在 0..6 返回 3 (`INVALID_VALUE`)；ADS1299 配置校验失败返回 5 (`APPLY_FAILED`)。OpenBCI 输出模式仅允许 24 倍，其他档返回 6 (`UNSUPPORTED`)。

OpenBCI 兼容数据包不携带增益元数据。要配置或读取可变增益，请使用 BCIKit 原生协议。
