### 微缩车模控制程序说明

本目录为基于 STM32F103 的微缩车模控制工程（Keil/MDK 工程）。代码以标准外设库为基础，实现了**循迹（黑线识别）+ 双闭环速度/位置控制 + 姿态/陀螺仪补偿**的整车控制逻辑。

---

### 目录结构概览

- **根目录**
  - `Project.uvprojx` / `Project.uvoptx` / `Project.uvguix.*`：Keil 工程文件。
  - `JLinkSettings.ini` / `JLinkLog.txt` / `EventRecorderStub.scvd`：仿真/下载相关配置与日志。
  - `keilkill.bat`：清理 Keil 生成文件的脚本。

- **`Start/` 启动与内核相关**
  - `startup_stm32f10x_*.s`：启动文件，中断向量表与复位入口。
  - `system_stm32f10x.c` / `.h`、`core_cm3.*`：系统时钟、内核相关配置。

- **`System/` 通用系统工具**
  - `Delay.c` / `.h`：毫秒/微秒级延时函数，实现基于定时器或 SysTick 的延时。

- **`Library/` STM32 标准外设库**
  - `stm32f10x_*.c` / `.h`、`misc.c` 等：GPIO、ADC、TIM、USART、RTC、FSMC、DMA 等外设驱动实现。
  - 本工程的大部分底层外设操作（GPIO、定时器、SPI、ADC 等）都通过这些库函数完成。

- **`User/` 用户应用与控制逻辑（核心控制模块）**
  - 该目录下每个 `.c/.h` 文件都是一个相对独立的功能模块，详见下一节。

- **`Objects/`、`Listings/`、`DebugConfig/`**
  - 工程编译生成的中间文件、列表文件以及调试配置，一般无需手动修改。

---

### 控制模块说明（`User/` 目录）

**总体说明**：  
用户控制逻辑集中在 `User` 目录，由 `main.c` 作为入口，`stm32f10x_it.c` 中的 `SysTick_Handler` 作为**核心周期调度函数**。各模块之间通过全局变量和函数调用协同工作，形成“**传感器采样 → 线路识别 → 双环 PID 控制 → 电机驱动**”的闭环控制链路。

- **`main.c`（主程序入口）**
  - 初始化所有外设与功能模块：RGB 灯、OLED 屏幕、IMU（`LSM6DSR`）、PWM（`M3PWM`）、电机控制（`Motor_ctr`）、编码器（`ABEncoder`）、多路 ADC（`ADC_get`）、按键（`Key_Scan`）、串口（`Uart_Config`）、循迹模块（`BlackPoint_Finder`）、PID 控制器（`PID_Controller`）等。
  - 配置 `SysTick` 周期中断，并启动 PWM 输出、电机等。
  - 在主循环中：
    - 调用 `Key_Scan_Update()` 读取并解析按键事件；
    - 根据按键事件设置 RGB 颜色、启停电机（`Motor_Enable()`）与基础占空比（例如 `M3PWM_SetDutyCycle(950)`），通过 `star_car` 标志控制整车是否运行；
    - 实时在 OLED 上显示位置偏差、车载电压 `BDI_V` 以及左右轮速度和等信息。

- **`stm32f10x_it.c`（中断与周期调度）**
  - 保留了 Cortex-M3 标准异常处理模板，真正与控制逻辑相关的是 `SysTick_Handler`：
    - 周期性读取 IMU 数据：`LSM6DSR_ReadData()` → `LSM6DSR_ConvertToPhysics()`，并对角速度 `gy_rads` 做时间积分得到 `add_angle` 等量。
    - 利用 `uart_rev_tiem` 实现通讯看门狗：若超过一定时间未接收数据，则关闭 PWM、禁止电机、清零 `star_car`。
    - 调用 `ABEncoder_UpdateSpeed()` 获取轮速；调用 `MuxADC_SampleAll()` 采集 16 路光电传感器 ADC 值。
    - 调用 `BlackPoint_Finder_Search(g_mux_adc_values, &result_BlackPoint)`：对 16 路传感器数据进行归一化、阈值与形态判断，计算黑线精确位置 `result_BlackPoint.precise_position`；若找到黑线则更新全局 `position_get`（以 0.1 点为单位），若长时间丢线则逐步累积 `lose_time`，最终触发电机关闭与停车保护。
    - 最后调用 `PID_Control_Update()` 完成一次完整的双环控制计算并更新电机输出。

- **`Motor_ctr.c` / `.h`（电机驱动与频率控制）**
  - 管理两路直流电机的**使能、方向和 PWM 占空比**：
    - 使用 `TIM1` 作为 PWM 定时器，`PA9`、`PA11` 为 PWM 输出，`PA8`、`PA10` 为方向控制，`PA12` 为电机总使能。
    - `Motor_Init()`：配置 GPIO、TIM1，设置默认 PWM 频率（约 17kHz）并启动计数器；默认关闭电机并占空比为 0。
    - `Motor_Enable()` / `Motor_Disable()`：通过拉高/拉低 `PA12` 控制电机供电。
    - `Motor_SetSpeed(motor_id, duty)`：按 0–10000 线性映射到 `TIM1->ARR`，设置指定电机的占空比。
    - `Motor_SetDirection(motor_id, direction)`：控制正/反转。
    - `Motor_Stop()` / `Motor_StopAll()`：快速停止指定或者全部电机。
    - `Motor_SetFrequency(freq)`：根据期望频率重新计算 PSC/ARR，并保持当前占空比不变。

- **`PID_Controller.c` / `.h`（双环 PID 控制）**
  - **速度环** `SpeedPID_*`：
    - `SpeedPID_Init()`：设置 Kp/Ki/Kd、输出限幅与积分限幅。
    - `SpeedPID_Calculate(target_speed, current_speed)`：使用增量式 PID 算法输出基础速度控制量，结果记为 `speed_output`。
  - **位置环** `PositionPID_*`：
    - `PositionPID_Init()`：设置 Kp/Ki/Kd、陀螺仪补偿系数 `gyro_kd`、输出/积分限幅与目标位置 `target_position`。
    - `PositionPID_Calculate(current_position)`：基于当前位置与目标位置计算偏差；积分抗饱和；利用 `LSE6DSR_data.gz_rads` 进行姿态/角速度补偿，输出左右轮差速修正量 `position_correction`。
  - **电机速度与方向统一接口**：
    - `Motor_SetSpeedWithDirection(motor_id, speed_target)`：根据正负号自动设置前进/后退方向，并限制在 `MOTOR_DUTY_MAX` 范围内。
  - **整体 PID 调度** `PID_Control_Update()`：
    - 将 `position_get` 转为实际位置 `current_position`；
    - 位置环计算得到 `position_correction`；
    - 使用 `speed_left`、`speed_right` 求平均速度 `avg_speed`；
    - 动态计算速度目标 `i_speed`（按与目标位置的偏差自适应调整速度）；并在车辆启动初期以 `statr_speed` 平滑爬升，避免瞬间过冲。
    - 若 `star_car` 置位，则执行速度环计算得到 `speed_output`；若未启动则复位 PID 内部状态、输出零速度；
    - 最终组合左右轮输出：  
      - 左轮：`left_output = speed_output + position_correction`  
      - 右轮：`right_output = speed_output - position_correction`  
      通过 `Motor_SetSpeedWithDirection()` 下发到实际电机。

- **`BlackPoint_Finder.c` / `.h`（黑线识别与精确位置计算）**
  - `BlackPoint_Finder_Init()`：为 16 路光电传感器分别设置经标定的最小/最大值，用于之后的归一化。
  - 提供配置与读取接口：`BlackPoint_Finder_SetSensorConfig()` / `GetSensorConfig()`。
  - `BlackPoint_Finder_IsBlackPoint()`：根据归一化阈值判断单个传感器是否处于黑线区域。
  - `BlackPoint_Finder_Search(adc_values, result)`：
    - 对 16 路 ADC 值按各自 `min/max` 做归一化（0=最黑，1=最白），并计算总和与最小值；
    - 通过剔除目标点及其左右邻点的方式计算“背景平均值”，再比对阈值（例如 `min_normalized/other_average` 与 `other_average` 的约束）判断是否真正存在有效黑线；
    - 若通过判定，则对最小值及其相邻两点使用**加权重心插值算法**计算亚像素级精确位置 `precise_position`，并限制在 `[0, SENSOR_COUNT-1]` 范围内；
    - 将结果写入 `BlackPointResult_t`（`found`、`position`、`precise_position`）并缓存为 `last_position`/`last_precise_position`，供丢线时继续使用。

- **`LSM6DSR_Config.c` / `.h`（IMU 驱动与物理量转换）**
  - 提供软/硬件 SPI 两套实现，宏 `USE_SOFTWARE_SPI` 控制选择。
  - `LSM6DSR_Init()`：初始化 GPIO、SPI2 或软 SPI，读取 `WHO_AM_I` 校验设备 ID，并依次配置加速度计和陀螺仪：
    - 加速度计：52Hz，±2g，开启 X/Y/Z 轴，高性能模式；
    - 陀螺仪：±2000 dps，开启 X/Y/Z 轴，并配置中断等。
  - `LSM6DSR_ReadData()`：连续读取 6 轴原始数据（gx/gy/gz, ax/ay/az）。
  - `LSM6DSR_ConvertToPhysics()`：将原始数据转换为：
    - 加速度 \(g\) 单位（`ax_g/ay_g/az_g`）；
    - 角速度弧度每秒（`gx_rads/gy_rads/gz_rads`），包含零偏补偿与比例系数。
  - 全局结构体 `LSE6DSR_data` 在 `SysTick_Handler` 中被周期性更新，并用于姿态积分与位置环中陀螺仪补偿。

- **`ABEncoder.c` / `.h`（编码器测速）**
  - 通过外部中断/定时器计数实现左右轮编码器计数，并计算 `speed_left`、`speed_right`。
  - `ABEncoder_UpdateSpeed()` 在 `SysTick_Handler` 中周期调用，为 PID 控制提供速度反馈。

- **`ADC_get.c` / `.h`（多路 ADC 采集）**
  - 实现光电传感器的多路轮询采样（例如通过多路复用器 + ADC 通道）。
  - `MuxADC_Init()`：初始化 ADC 与相关通道。
  - `MuxADC_SampleAll()`：采集全部 16 路数据到全局数组 `g_mux_adc_values[]`，供 `BlackPoint_Finder` 使用。

- **`Key_Scan.c` / `.h`（按键扫描与事件管理）**
  - 使用定期调用方式进行去抖与状态机管理。
  - `Key_Scan_Init()`：初始化按键 GPIO。
  - `Key_Scan_Update()`：在主循环中周期调用，更新按键状态。
  - `Key_GetEvent()`：返回去抖后的按键事件（如 `KEY_K1`–`KEY_K4`），主循环根据事件来启动/停止小车或切换模式。

- **`M3PWM.c` / `.h`（PWM 基础模块）**
  - 提供更通用的 PWM 初始化与占空比设置接口。
  - `M3PWM_Init()`：配置定时器与输出通道。
  - `M3PWM_Start()`：启动 PWM 计数与输出。
  - `M3PWM_SetDutyCycle(value)`：设置全局 PWM 占空比（用于整体功率调节/限速）。

- **`OLED.c` / `.h`（OLED 显示驱动）**
  - 初始化 OLED 屏幕并提供基础显示接口，如 `OLED_ShowSignedNum()` 等。
  - 当前用于显示位置偏差、电压、速度等调试/运行信息。

- **`RGB_Led.c` / `.h`（RGB 指示灯）**
  - 控制板载 RGB 三色 LED，用于指示当前工作状态或模式。
  - `RGB_Init()` 完成 GPIO 初始化；`RGB_SetColor(x)` 根据不同按键/状态显示不同颜色。

- **`Uart_Config.c` / `.h`（串口配置与通讯）**
  - `Uart2_Init(115200)`：初始化 USART2，波特率 115200。
  - 与 `uart_rev_tiem` 配合，用于通讯超时检测；若长时间未接收数据则触发安全停机。

- **`pose.c` / `.h`（姿态/位姿相关辅助）**
  - 封装与姿态解算相关的数据结构与接口（当前工程中预留了 `prepare_data()`、`imuupdate()` 等调用位置，便于后续扩展为完整姿态解算）。

---

### 整体控制逻辑（运行流程）

1. **上电与初始化**
   - 复位后进入 `main()`，依次完成 RGB/OLED/IMU/PWM/电机/编码器/ADC/按键/串口/循迹/PID 等模块初始化。
   - 配置 `SysTick` 以固定周期中断（工程中设为约 1 ms 级），作为系统“心跳”。

2. **主循环（人机交互与状态显示）**
   - 持续调用 `Key_Scan_Update()` 与 `Key_GetEvent()`，根据按键事件：
     - 设置 RGB 指示颜色；
     - 通过 `Motor_Enable()` 与 `M3PWM_SetDutyCycle()` 启动整车；
     - 置位 `star_car` 以允许 PID 输出生效。
   - 使用 OLED 显示 `position_get`、`BDI_V`、左右轮速度等运行参数，方便调试与监控。

3. **SysTick 周期中断（核心控制环）**
   - 读取 IMU 原始数据并转换为物理量，更新全局姿态/角速度；
   - 更新编码器速度与 ADC 采样值；
   - 调用 `BlackPoint_Finder_Search()` 计算黑线精确位置与是否存在有效线段，更新 `position_get`；
   - 执行串口看门狗与丢线保护逻辑，必要时关闭 PWM、电机并清零 `star_car`；
   - 调用 `PID_Control_Update()`：
     - 位置环根据 `position_get` 输出差速修正量；
     - 速度环根据期望车速与平均实测速计算基础速度输出，同时做平滑加速；
     - 综合结果后调用 `Motor_SetSpeedWithDirection()` 更新左右电机占空比与方向。

4. **异常与安全保护**
   - 当长时间未检测到黑线（`lose_time` 超出阈值）或串口超时（`uart_rev_tiem` 超出阈值）时：
     - 通过 `Motor_Disable()` 与 `M3PWM_SetDutyCycle(0)` 停止电机；
     - 清零 `star_car` 防止 PID 继续输出。
   - 出现硬件故障（HardFault、BusFault 等）时，中断处理函数进入死循环，方便调试定位。

---

### 快速上手与移植建议

- **编译与下载**：使用 Keil/MDK 打开 `Project.uvprojx`，根据实际硬件（晶振、下载器等）适当调整工程配置后即可编译、下载。
- **硬件适配**：如需更改电机、传感器或 IMU 接口，请同步修改：
  - 定时器与 GPIO 映射（`Motor_ctr.c`、`M3PWM.c`、`ABEncoder.c`）；
  - ADC 通道/倍压与标定参数（`ADC_get.c`、`BlackPoint_Finder.c` 中 `min/max`）；
  - IMU 通信引脚与寄存器配置（`LSM6DSR_Config.c`）。
- **控制策略调整**：
  - 车速与循迹灵敏度可通过修改 `PID_Init()` 中的 Kp/Ki/Kd 与 `gyro_kd`、`target_position` 等参数实现；
  - `BlackPoint_Finder` 内的阈值比例、权重算法也可根据赛道反射特性和传感器差异做进一步优化。

通过以上结构和说明，可以快速理解本文件夹中**所有控制模块及其逻辑关系**，并在此基础上进行二次开发、调参或移植到其他车模平台。

