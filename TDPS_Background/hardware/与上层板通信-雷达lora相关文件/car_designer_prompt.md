# STM32 主控 ↔ ESP32S3-1 通信协议说明（供小车设计者）

## 硬件连接

| STM32 侧 | ESP32S3-1 侧 | 说明 |
|---------|-------------|------|
| UART TX | GPIO18 (Comm-3) | STM32 → ESP32S3-1 |
| UART RX | GPIO17 (Comm-2) | ESP32S3-1 → STM32 |
| GND | Comm-1 / Comm-4 | 共地 |

- 波特率：**115200 8N1**（无流控）
- 连接器：PCB 上 4P 2.54mm 排针（Comm）
- 电平：3.3V TTL

## 帧格式

所有通信使用统一的二进制帧格式：

```
A5 5A | ver(1B) | type(1B) | seq(2B LE) | payload_len(2B LE) | payload(N bytes) | checksum(1B)
```

- `A5 5A`：双字节帧头
- `ver`：协议版本号，固定 `0x01`
- `type`：帧类型（见下表）
- `seq`：16-bit 自增序号（LE），发送方维护，用于检测丢帧
- `payload_len`：payload 字节数（LE）
- `checksum`：从 ver 到 payload 末字节的逐字节 XOR

## 命令与响应

### STM32 → ESP32S3-1 请求（type 0x01–0x04）

| Type | 名称 | Payload | 含义 |
|------|------|---------|------|
| 0x01 | REQ_DECISION | 无 | 请求障碍物通行方向决策 |
| 0x02 | REQ_RADAR | 无 | 请求雷达原始数据 |
| 0x03 | AT_POSITION | 无 | **通知已到达检测位置 + 请求决策**（推荐使用） |
| 0x04 | PING | 无 | 心跳检测 |

### ESP32S3-1 → STM32 响应（type 0x11–0x20）

| Type | 名称 | Payload 长度 | 含义 |
|------|------|-------------|------|
| 0x11 | DECISION | 2 字节 | 障碍物通行方向 |
| 0x12 | RADAR | 21 字节 | 雷达数据 |
| 0x20 | STATUS | — | 状态心跳（预留） |

## Payload 详解

### DECISION（2 字节）

```
decision(1B) | radar_presence(1B)
```

| 字段 | 值 | 含义 |
|------|-----|------|
| decision | 0 = GO_LEFT | **从左侧通过**（右侧有障碍） |
| decision | 1 = GO_RIGHT | **从右侧通过**（左侧有障碍） |
| decision | 2 = UNKNOWN | 无法判断（传感器故障） |
| radar_presence | 0 / 1 | 雷达原始检测结果 |

**决策逻辑**：雷达安装在车体右侧。雷达检测到障碍物 → 障碍物在右侧 → 建议从左侧通过（GO_LEFT）。

### RADAR（21 字节）

```
timestamp_ms(4B LE) | presence(1B) | distance(2B LE) | energy[8](16B)
```

| 字段 | 说明 |
|------|------|
| timestamp_ms | 硬件时间戳 ms |
| presence | 0=无人, 1=检测到目标 |
| distance | 目标距离值 |
| energy[8] | 前 8 个距离门能量值（每门 2B LE，覆盖 0–5.6m） |

## 典型通信流程

### Task 3：障碍物检测

```
STM32                              ESP32S3-1
  │                                    │
  │  小车到达检测位置                   │
  │                                    │
  │── AT_POSITION (0x03, payload=0) ──→│
  │                                    │ 查询雷达 + 计算决策
  │←─ DECISION (0x11, 2B payload) ────│  GO_LEFT / GO_RIGHT
  │                                    │
  │  小车根据决策选择路径通过           │
  │                                    │
```

### 获取雷达原始数据（可选）

```
STM32                              ESP32S3-1
  │                                    │
  │── REQ_RADAR (0x02, payload=0) ───→│
  │←─ RADAR (0x12, 21B payload) ─────│  presence + distance + energy
  │                                    │
```

### 心跳检测

```
STM32                              ESP32S3-1
  │── PING (0x04) ───────────────────→│
  │   （ESP32S3-1 在日志中记录）        │
```

## 赛场操作流程

1. **上电**：ESP32S3-1 自动完成 WiFi 授时 → LoRa 配置 → 连接 ESP32S3-2
2. **发令**：发令枪响后，STM32 检测到小车开始运动，ESP32S3-1 从此时计时（elapsed time）
3. **Task 2 Arch 2.1**：小车通过第一个拱门时，ESP32S3-2 自动检测并通过 LoRa 发送
4. **Task 3 检测点**：STM32 到达大矩形区域前，发送 `AT_POSITION(0x03)` → ESP32S3-1 回复通行方向
5. **Task 2 Arch 2.2**：小车通过第二个拱门时自动 LoRa 发送
6. **终点**：到达 Finish

## 注意事项

1. **帧同步**：接收端需按 `A5 5A` 帧头逐字节搜索，不假设帧之间无垃圾数据
2. **序列号**：seq 从 0 开始自增，到达 65535 后 wraparound 到 0
3. **校验和**：每次收到帧后验证 XOR checksum，不匹配则丢弃
4. **超时**：发送请求后若 500ms 未收到回复可重发一次
5. **BLE 覆盖**：手机可通过 BLE 指令强制指定雷达检测结果（RADAR_LEFT/RADAR_RIGHT），决策会相应改变

---

