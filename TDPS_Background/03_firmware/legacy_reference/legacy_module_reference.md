# 旧模块结构与命名备查

本文只作为旧工程结构备查，不代表当前主用固件。旧命名在新硬件迁移后容易造成误解，因此当前源码已改为新模块名。

## 1. 旧 IMU 模块

旧文件名：

- `User/LSM6DSR_Config.c`
- `User/LSM6DSR_Config.h`

旧接口名：

- `LSM6DSR_Init`
- `LSM6DSR_ReadID`
- `LSM6DSR_ReadData`
- `LSM6DSR_ConvertToPhysics`
- `LSM6DSR_data`

当前替代：

- `User/MPU6050_Config.c`
- `User/MPU6050_Config.h`
- `MPU6050_Init`
- `MPU6050_ReadID`
- `MPU6050_ReadData`
- `MPU6050_ConvertToPhysics`
- `MPU6050_data`

迁移原因：新主板使用 MPU6050 I2C 模块，不再使用旧 LSM6DSR SPI 模块。

## 2. 旧光电/ADC 模块

旧文件名：

- `User/ADC_get.c`
- `User/ADC_get.h`

旧接口/变量名：

- `MuxADC_Init`
- `MuxADC_ReadChannel`
- `MuxADC_SampleAll`
- `g_mux_adc_values`

当前替代：

- `User/LineSensor.c`
- `User/LineSensor.h`
- `LineSensor_Init`
- `LineSensor_ReadChannel`
- `LineSensor_SampleAll`
- `g_line_sensor_values`
- `g_battery_adc_value`

迁移原因：新光电板是 7 路直连数字输入，不再是旧 16 路复用 ADC；电池 PA0 ADC 单独保存到 `g_battery_adc_value`。

## 3. 旧风扇 PWM 模块

旧文件名：

- `User/M3PWM.c`
- `User/M3PWM.h`

旧接口名：

- `M3PWM_Init`
- `M3PWM_Start`
- `M3PWM_Stop`
- `M3PWM_SetDutyCycle`
- `M3PWM_GetDutyCycle`

当前替代：

- `User/FanMotor_SafetyLock.c`
- `User/FanMotor_SafetyLock.h`
- `FanMotor_SafetyLock_Init`
- `FanMotor_SafetyLock_ForceOff`
- `FanMotor_SafetyLock_GetDutyCycle`

迁移原因：当前固件处于硬件排查安全锁状态，不允许风扇 PWM 输出；旧 `M3PWM` 名称会误导为仍可直接输出 PWM。

## 4. 旧结构保留原则

- 旧命名只在本文档中保留，用于对照历史代码和问题记录。
- 当前 `User/` 源码目录不应再出现旧 `.c/.h` 文件。
- 如果文件浏览器里看到 `LSM6DSR_Config.o`、`ADC_get.o`、`M3PWM.o`，它们只是旧编译产物，不是当前源码，应在安全时清理。