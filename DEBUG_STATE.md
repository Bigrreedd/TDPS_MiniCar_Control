# USART3 调试串口 + 速度 PID 整定记录

> 最后更新：2026-06-03 16:20
> 上位机分支：`upper-test`，下位机分支：`lower-pid`

---

## 1. 硬件接线

| STM32 上板 | USB-TTL 模块 |
|---|---|
| PB10 (USART3_TX) | RX |
| PB11 (USART3_RX) | TX |
| GND | GND |

**115200 8N1**，printf 重定向到 USART3。

---

## 2. 编译开关

全部定义在 `User/stm32f10x_conf.h`：

```c
#define USART3_DEBUG_ON_PB10  1   // 1=PB10/PB11 用于 USART3，ch6 停用（6 路灰度）
                                   // 0=PB10 归还 ch6（7 路灰度），USART3 不启用
```

联动影响：
- `SENSOR_COUNT`：1→6，0→7（`BlackPoint_Finder.h`）
- PB10/PB11 GPIO 初始化：仅 =1 时配置（`Uart_Config.c`）
- ch6 读数：=1 时返回固定白色值（`LineSensor.c`）
- 遥测输出：=1 时通过 USART3 发送（`main.c`）

---

## 3. 速度环 PID 参数（自动缩放）

**基准增益**（在参考速度 80 cnt/s 下整定）：
```c
SpeedPID_Init(&g_speed_pid, 2.0f, 14.0f, 2.0f, 8000.0f, -8000.0f);
//                          Kp    Ki     Kd    max      min
```

**运行时自动缩放：** 每个控制周期根据 `i_speed` 动态调整增益：
```
scale = i_speed / 80.0, clamped [0.3, 1.5]
Kp = 2.0 × scale, Ki = 14.0 × scale, Kd = 2.0 × scale
```

| 目标速度 | scale | Kp | Ki | Kd | 场景 |
|---|---|---|---|---|---|
| 50 cnt/s | 0.625 | 1.25 | 8.75 | 1.25 | 低速弯道 |
| 80 cnt/s | 1.0 | 2.0 | 14.0 | 2.0 | 基准直线 |
| 120 cnt/s | 1.5 | 3.0 | 21.0 | 3.0 | 高速直道 |

| 参数 | 基准值 | 说明 |
|---|---|---|
| Kp | 2 | 增量式，作用于 Δe，压低防振荡 |
| Ki | 14 | 增量式，作用于 e(k)，主力驱动 |
| Kd | 2 | 阻尼项，200ms 窗口下避免放大噪声 |
| output_max/min | ±8000 | PID 内部限幅 |
| Δu clamp | ±500 | 每步增量限幅，防启动猛冲 |

**速度窗口：** `SPEED_WIN_MS = 200`（200ms 滑动窗口，1 tick = 5 cnt/s）

**反向制动：** `CLOSED_LOOP_REVERSE_ENABLE = 1`（允许 PID 负输出制动）

**死区补偿：** `MOTOR_DEADZONE_L=750`, `MOTOR_DEADZONE_R=950`

---

## 4. 位置环（角度环）PID 参数

```c
PositionPID_Init(&g_position_pid, 180.0f, 0.0f, 200.0f, 0.0f, 9000.0f, -9000.0f, 2.5f);
//                                   Kp     Ki   Kd     gyro   max      min      target
```

| 参数 | 值 | 说明 |
|---|---|---|
| Kp | 180 | 位置式，作用于 position error |
| Ki | 0 | 循迹不需要位置积分 |
| Kd | 200 | 抑制抖动 |
| target | 2.5 | 6 路灰度中心（SENSOR_COUNT=6）|

### 速度自适应缩放

**物理原理：** `a_lat ∝ position_correction × V`

保持横向加速度恒定 → `position_correction_effective = position_raw × V0 / V`

| 车速 | 缩放系数 | 效果 |
|---|---|---|
| ≤60 cnt/s | 1.00× | 全量修正 |
| 80 cnt/s | 0.75× | |
| 120 cnt/s | 0.50× | 下限 0.35× |

---

## 5. 轮速平衡

```c
#define WHEEL_BALANCE_KP    8.0f     // 左右轮速差增益
#define WHEEL_BALANCE_LIMIT 300.0f   // 限幅
```

**注意：** 台架模式（`BENCH_FIXED_SPEED_ENABLE=1`）下轮速平衡被禁用，上路后自动生效。

---

## 6. 遥测格式

```
L=<左速> R=<右速> T=<目标> out=<速度环输出> dl=<左轮目标> dr=<右轮目标> S=<ch0>,<ch1>,<ch2>,<ch3>,<ch4>,<ch5>
```

| 字段 | 单位 | 说明 |
|---|---|---|
| L/R | cnt/s | 编码器速度，200ms 窗口换算 |
| T | cnt/s | 当前目标速度 |
| out | 占空比 | g_speed_pid.last_output |
| dl/dr | 占空比 | 最终电机目标（速度+位置+平衡）|
| S | 0~4095 | 6 路灰度（0=黑, 4095=白），ch2/ch3 为正中两路 |

**刷新率：** ~300ms/帧（跟随 OLED 遥测刷新）

---

## 7. 调参历程

| 阶段 | Kp | Ki | Kd | 问题 | 解决 |
|---|---|---|---|---|---|
| 原始 | 5.5 | 5.1 | 5.8 | Ki≈Kp，积分风up，太慢 | — |
| V1 | 25 | 1.5 | 15 | 小幅振荡 ±265，速度 60-80 | 最佳基础 |
| V2 | 12 | 2.0 | 30 | 崩溃，Kd 放大噪声 | 回退 |
| V3 | 15 | 3.0 | 5 | 大幅振荡 | 回退 |
| V4 | 8 | 4.0 | 3 | 反向制动过猛 | 启用 CLOSED_LOOP_REVERSE_ENABLE |
| V5 | 3 | 5.0 | 1 | 速度 50→80 收敛，负速 -6~-16 | — |
| V6 | 2 | 7.0 | 2 | 速度 43→76，输出 102→431，稳态良好 | — |
| V7 | 2 | 8.0 | 2 | start_speed ramp 导致启动反转 | 砍 ramp |
| V8 | 2 | 12.0 | 2 | 稳态 73-76，启动 116+ 偏快 | 加 Δu clamp ±500 |
| V9 | 2 | 14.0 | 2 | 稳态 73-86，启动 90→70 平滑 | **当前** |

---

## 8. 当前已知问题

1. **台架左右轮速偏差 10-16 cnt/s**：右轮偏快，由死区补偿差异（R=950 vs L=750）导致。WHEEL_BALANCE_KP=8 上路后自动修正。
2. **台架模式位置环无意义**：`BENCH_FIXED_SPEED_ENABLE=1`，Path 状态机被跳过，上路前需置 0。
3. **上路前需切换：**
   ```c
   #define BENCH_FIXED_SPEED_ENABLE 0   // 启用 Path 状态机
   ```

---

## 9. 文件改动清单（upper-test 分支）

| 文件 | 改动 |
|---|---|
| `User/stm32f10x_conf.h` | `USART3_DEBUG_ON_PB10 1` 全局开关 |
| `User/BlackPoint_Finder.h` | `SENSOR_COUNT` 联动开关（6/7）|
| `User/LineSensor.c` | ch6 条件禁用 |
| `User/Uart_Config.h/.c` | USART3 实现 + printf 重定向 |
| `User/main.c` | USART3 初始化、遥测输出、SPEED_WIN_MS=200、CLOSED_LOOP_REVERSE_ENABLE=1 |
| `User/PID_Controller.c` | PID 参数整定、砍 ramp、Δu clamp、位置环启用、速度自适应 |
