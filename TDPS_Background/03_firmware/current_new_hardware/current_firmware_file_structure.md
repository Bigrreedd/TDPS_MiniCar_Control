# 当前新硬件固件文件结构说明

本文记录当前固件迁移到新主板、新 4Pin I2C OLED、MPU6050 I2C IMU、7 路直连光电板和风扇安全锁后的主用文件结构。

## 1. 当前主用模块

| 功能 | 当前文件 | 当前接口/变量 | 说明 |
| --- | --- | --- | --- |
| OLED 4Pin I2C 显示 | `User/OLED.c`, `User/OLED.h` | `OLED_Init`, `OLED_Show*` | PA15=SCL，PB12=SDA，软件 I2C，关闭 JTAG 释放 PA15 |
| 中文 OLED 遥测 | `User/TelemetryScreen.c`, `User/TelemetryScreen.h` | `TelemetryScreen_Update` | 光电掩码按 7 路显示为 2 位十六进制 |
| RGB LED | `User/RGB_Led.c`, `User/RGB_Led.h` | `RGB_*` | PB5=R，PB4=G，PB3=B，关闭 JTAG 释放 PB3/PB4 |
| MPU6050 IMU | `User/MPU6050_Config.c`, `User/MPU6050_Config.h` | `MPU6050_Init`, `MPU6050_ReadData`, `MPU6050_data` | PB8=SCL，PB9=SDA，软件 I2C |
| 7 路直连光电 + 电池 ADC | `User/LineSensor.c`, `User/LineSensor.h` | `LineSensor_Init`, `LineSensor_SampleAll`, `g_line_sensor_values`, `g_battery_adc_value` | S1~S7 为数字输入，PA0 为电池 ADC |
| 黑点识别 | `User/BlackPoint_Finder.c`, `User/BlackPoint_Finder.h` | `BlackPoint_Finder_Search` | `SENSOR_COUNT=7` |
| 风扇安全锁 | `User/FanMotor_SafetyLock.c`, `User/FanMotor_SafetyLock.h` | `FanMotor_SafetyLock_Init`, `FanMotor_SafetyLock_ForceOff`, `FanMotor_SafetyLock_GetDutyCycle` | 当前不配置 TIM2/PB11，不输出 PWM，只保留占空比为 0 的状态接口 |
| 按键扫描 | `User/Key_Scan.c`, `User/Key_Scan.h` | `Key_Scan_Init`, `Key_Scan_Update` | K1=PB14，K2=PB13，K3=PC14，K4=PC13 |
| 主控流程 | `User/main.c` | 主循环、遥测、控制安全锁 | 只允许安全停机逻辑，不重新启动车/风扇 |
| SysTick 中断 | `User/stm32f10x_it.c` | `SysTick_Handler` | 周期读取 MPU6050，执行安全停机看门狗 |

## 2. 当前引脚分配

| 外设 | 引脚 |
| --- | --- |
| OLED SCL/SDA | PA15 / PB12 |
| RGB R/G/B | PB5 / PB4 / PB3 |
| MPU6050 SCL/SDA | PB8 / PB9 |
| 光电 S1~S7 | PA1, PA4, PA5, PB0, PB1, PB2, PB10 |
| 电池 ADC | PA0 / ADC12_IN0 |
| 按键 K1~K4 | PB14, PB13, PC14, PC13 |
| 电机使能/方向/PWM | PA12, PA8, PA10, PA9, PA11 |
| 编码器 | PA6, PA7, PB6, PB7 |
| 串口 USART2 | PA2 TX, PA3 RX |

## 3. 当前安全状态

当前工程处于硬件排查安全锁状态：

- `FanMotor_SafetyLock.c` 不配置 TIM2，也不配置 PB11 PWM 输出。
- `FanMotor_SafetyLock_ForceOff()` 只把风扇占空比状态保持为 0。
- `main.c` 不应调用 `Motor_Enable()`。
- `main.c` 不应把 `is_racing` 或 `g_manual_drive_active` 置为 1。
- 所有旧 `M3PWM_*` 接口已替换为 `FanMotor_SafetyLock_*`。
- `main.c` 当前使用 `SafetyStopAll()` 作为统一停机入口，K1/K2/K3/K4、LoRa 速度/停止指令和主循环控制 tick 均不会启动电机或风扇。
- 旧的风扇测试占空比宏、直线测试宏和轮速测试辅助函数已从 `main.c` 移除，避免安全锁状态下残留可误触发的测试路径。
- 占空比上限计算见 `TDPS_Background/hardware/motor_pwm_duty_limit_analysis.md`；当前轮子电机保持 `MOTOR_DUTY_SAFE_MAX = 2000`，当前风扇保持 `FAN_MOTOR_SAFE_DUTY_OFF = 0`。

## 4. 当前 Keil 工程引用

`Project.uvprojx` 当前应引用：

- `User/MPU6050_Config.c/.h`
- `User/LineSensor.c/.h`
- `User/FanMotor_SafetyLock.c/.h`

不应再引用：

- `User/LSM6DSR_Config.c/.h`
- `User/ADC_get.c/.h`
- `User/M3PWM.c/.h`

## 5. 当前仍需实物确认

- OLED 地址是否为 `0x3C`，当前驱动发送 8-bit 地址 `0x78`。
- MPU6050 `WHO_AM_I` 是否返回 `0x68` 或 `0x69`。
- 光电模块黑线有效电平是否确认为低电平。
- PA15/PB3/PB4 关闭 JTAG 后是否正常工作，SWD 是否仍可连接。
- PB10 光电输入是否无其它外设复用干扰。