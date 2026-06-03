# 新主板外设引脚迁移说明

本文记录当前固件已经迁移到新主板、新 4Pin I2C OLED、新 MPU6050 模块和 7 路直连光电板后的引脚分配与软件约束。

## 1. OLED 模块

新 OLED 为经典 4Pin I2C OLED 模块。

| 信号 | STM32 引脚 | 说明 |
| --- | --- | --- |
| SCL | PA15 | 软件 I2C 时钟，需要关闭 JTAG 后释放 |
| SDA | PB12 | 软件 I2C 数据 |

固件位置：

- `User/OLED.c`

注意：PA15 默认为 JTAG 相关引脚，固件在 OLED/RGB 初始化中调用 `GPIO_Remap_SWJ_JTAGDisable`，关闭 JTAG、保留 SWD 调试。

## 2. RGB LED

为避开 OLED 的 PA15 和其它外设，新 RGB LED 引脚调整为：

| 颜色 | STM32 引脚 |
| --- | --- |
| R | PB5 |
| G | PB4 |
| B | PB3 |

固件位置：

- `User/RGB_Led.c`
- `User/RGB_Led.h`

注意：PB3/PB4 也是 JTAG 相关引脚，必须关闭 JTAG 后才能作为普通 GPIO 使用。

## 3. IMU 模块

新主板使用 MPU6050 I2C 模块，固件保留原 `LSM6DSR_*` 接口名以兼容上层控制代码。

| MPU6050 信号 | STM32 引脚 | 说明 |
| --- | --- | --- |
| SCL | PB8 | 软件 I2C 时钟 |
| SDA | PB9 | 软件 I2C 数据 |

固件位置：

- `User/LSM6DSR_Config.c`
- `User/LSM6DSR_Config.h`

当前实现要点：

- 支持 MPU6050 地址 `0x68/0x69` 对应的 8-bit 写地址 `0xD0/0xD2`。
- 读取 `WHO_AM_I`，正常应返回 `0x68` 或 `0x69`。
- 初始化失败后 `g_mpu6050_ready=0`，后续读数直接清零，避免在 SysTick 中反复 I2C 超时。
- 角速度量程配置为 `±1000 dps`，转换系数约 `1879.299 LSB/(rad/s)`。

## 4. 7 路直连光电板

新光电板不再使用旧 16 路 74HC4067 复用 ADC 方案，改为 7 路独立数字输入。

| 光电通道 | STM32 引脚 | 软件索引 |
| --- | --- | --- |
| S1 | PA1 | `g_mux_adc_values[0]` |
| S2 | PA4 | `g_mux_adc_values[1]` |
| S3 | PA5 | `g_mux_adc_values[2]` |
| S4 | PB0 | `g_mux_adc_values[3]` |
| S5 | PB1 | `g_mux_adc_values[4]` |
| S6 | PB2 | `g_mux_adc_values[5]` |
| S7 | PB10 | `g_mux_adc_values[6]` |

固件位置：

- `User/ADC_get.c`
- `User/ADC_get.h`
- `User/BlackPoint_Finder.c`
- `User/BlackPoint_Finder.h`

当前软件约定：

- `SENSOR_COUNT = 7`。
- 光电输入按低电平有效处理：低电平映射为 `0`，高电平映射为 `4095`。
- 为兼容原有寻线算法，仍使用 `g_mux_adc_values` 数组。
- `g_mux_adc_values[0..6]` 为 7 路光电。
- `g_mux_adc_values[7..15]` 清零，不参与算法。
- `g_mux_adc_values[16]` 保留为 PA0 电池 ADC。
- 串口 RAW 输出格式改为 `RAW7 ... PA0=...`。
- OLED 第二行“光:”显示 7 路掩码，范围 `00` 到 `7F`。

如果实测发现光电模块是高电平表示黑线，需要把 `User/ADC_get.c` 中的 `LINE_SENSOR_ACTIVE_LOW` 改为 `0`。

## 5. 按键

| 按键 | STM32 引脚 |
| --- | --- |
| K1 | PB14 |
| K2 | PB13 |
| K3 | PC14 |
| K4 | PC13 |

固件位置：

- `User/Key_Scan.c`

## 6. PID 和显示适配

- `User/PID_Controller.c` 中位置环目标中心改为 `(SENSOR_COUNT - 1) / 2.0f`，7 路时中心为 `3.0`。
- `User/TelemetryScreen.c` 中光电掩码显示改为 2 位十六进制。
- `User/main.c` 中动态光电诊断循环均按 `SENSOR_COUNT` 遍历。

## 7. 电机与风扇安全锁

由于此前风扇/电机测试存在烧毁风险，当前固件仍保持硬件排查安全锁。

安全锁要求：

- `FAN_MOTOR_TEST_START_DUTY = 0`。
- `FAN_MOTOR_TEST_HOLD_DUTY = 0`。
- `FAN_MOTOR_TEST_ON_K1 = 0`。
- `main.c` 中不应重新启用 `Motor_Enable()`。
- `main.c` 中不应把 `is_racing` 或 `g_manual_drive_active` 置为 `1`。
- 不应存在非零 `M3PWM_SetDutyCycle(...)` 调用。
- `M3PWM.c` 在安全锁状态下不配置 PB11/TIM2_CH4 输出，不执行 TIM2 重映射，所有占空比请求都会强制保持为 `0`。

刷写和上电前建议：

1. 断开电机、风扇和主电池。
2. 只保留安全供电、ST-Link 和必要的调试串口。
3. 先确认 OLED、按键、MPU6050、RAW7 串口数据正常。
4. 确认安全锁固件工作后，再单独制定受限电流的驱动测试方案。

## 8. 当前仍需实测确认

- OLED I2C 地址是否为常见 `0x3C`，当前驱动发送 8-bit 地址 `0x78`。
- MPU6050 `WHO_AM_I` 是否为 `0x68/0x69`。
- 7 路光电的黑线有效电平是否为低电平。
- PB10 作为光电 S7 时是否受其它代码影响；当前安全锁固件不执行 TIM2 部分重映射，不配置 TIM2_CH3/PB10，也不配置 TIM2_CH4/PB11。
- PA15/PB3/PB4 在关闭 JTAG 后是否正常工作，SWD 调试是否仍可连接。
