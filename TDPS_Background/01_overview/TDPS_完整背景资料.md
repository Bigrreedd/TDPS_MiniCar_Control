# TDPS 微缩车模项目 — 完整背景资料

> 本文档由 `README.md`、`map.md`、`advice.md`、`Github仓库.md` 合并而成，内容只增不减。

> 当前状态提示：本文档包含早期 16 路光电阵列、LSM6DSR、`M3PWM` 风扇 PWM、`Motor_Enable()` 启动车辆等历史描述，仅作为项目背景参考。当前固件已经迁移到 7 路数字光电传感器、MPU6050 软件 I2C 和风扇 `FanMotor_SafetyLock`，主循环/按键/遥控路径保持安全停机。电机 PWM 占空比上限以 `TDPS_Background/hardware/motor_pwm_duty_limit_analysis.md` 为准：轮子电机当前保持 `MOTOR_DUTY_SAFE_MAX = 2000`，风扇当前保持 `0 / 1000` 不输出 PWM。

---

## 第一部分：微缩车模控制程序说明

### 项目概述

本目录为基于 STM32F103 的微缩车模控制工程（Keil/MDK 工程）。代码以标准外设库为基础，实现了**循迹（黑线识别）+ 双闭环速度/位置控制 + 姿态/陀螺仪补偿**的整车控制逻辑。

- **课程**：UESTCHN3018 Team Design Project and Skills
- **目标**：微缩车模在 Patio 赛道上的自主循迹、通信与雷达避障
- **仓库地址**：`https://github.com/Bigrreedd/TDPS_MiniCar_Control.git`

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
用户控制逻辑集中在 `User` 目录，由 `main.c` 作为入口，`stm32f10x_it.c` 中的 `SysTick_Handler` 作为**核心周期调度函数**。各模块之间通过全局变量和函数调用协同工作，形成"**传感器采样 → 线路识别 → 双环 PID 控制 → 电机驱动**"的闭环控制链路。

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
    - 通过剔除目标点及其左右邻点的方式计算"背景平均值"，再比对阈值（例如 `min_normalized/other_average` 与 `other_average` 的约束）判断是否真正存在有效黑线；
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
   - 配置 `SysTick` 以固定周期中断（工程中设为约 1 ms 级），作为系统"心跳"。

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

---

## 第二部分：Patio 赛道地图详解

### 1. 赛道整体布局与对称性

* [cite_start]**基本结构**：赛道设计采用完全的轴对称镜像布局 [cite: 50]。
* [cite_start]**比赛形式**：两支队伍分别从赛道的两端同时出发进行比赛 [cite: 50]。
* [cite_start]**路面规格**：赛道为白底路面，中心铺设用于引导小车行驶的黑线 [cite: 95]。
* [cite_start]**引导线参数**：黑线的宽度恒定为 3cm [cite: 98]。
* [cite_start]**起始区域**：起点设有一个 50x50cm 的绿色方框 [cite: 98]。
* [cite_start]**空间尺寸**：根据赛道俯视图标注，主要直道长度包括 250cm 和 200cm 等路段，整体跨度由这些长直道及转弯区域累加而成 [cite: 56, 60, 64]。

### 2. 路向引导标识与 180° U 型回转弯 (Start -> 1.1)

在前往 1.1 掉头位的直线路径上，设有关键的视觉参考标识用于辅助定位。

* **路向引导标识 (Directional Markers)**：
    * [cite_start]**数量与位置**：在 1.1 掉头位前后的直道上各设有一个标识，总计两个 [cite: 56]。
    * **形态特征**：标识由一条中心直线和两根呈 45° 角斜向叉开的线条组成，线条末端带有横杠封头。
    * **方向指示**：
        * [cite_start]**去程标识**：叉向朝向掉头位方向（向上） [cite: 56]。
        * [cite_start]**回程标识**：叉向背向掉头位，朝向终点方向（向下） [cite: 56]。
* **180° U-Turn (节点 1.1)**：
    * [cite_start]**物理尺寸**：回转直径（黑线中心距）为 70cm [cite: 62]。
    * [cite_start]**回程长度**：完成掉头后的回程直线段垂直长度为 200cm [cite: 60]。
* **左右判别逻辑**：通过检测 1.1 掉头时的转向极性（顺时针为左半区，逆时针为右半区）锁定 `g_MapSide` 状态。

### 3. 双重 S 弯特征与节点过渡 (S-Bend & Transition: 1.2 -> 2.1)

该区域是小车完成 1.1 节点掉头后的首个精密变向区，包含 S 弯轨迹、水平位移补偿段以及首个功能触发点。

* **数量与几何结构**：包含**两个连续的 S 型弯道**。每个 S 型结构由两个方向相反、平滑衔接的圆弧组成。
* **详细尺寸**：
    * **垂直跨度**：两段 S 弯的总垂直投影距离为 120cm（由两个 60cm 的投影段组成）。
    * **水平位移**：该双 S 弯轨迹本身产生的总水平偏移量为 30.5cm。
* **镜像方向与极性特征**：
    * **左半区 (Left Half)**：回程下行时，呈现为**两个连续的反向 S 形**。引导线首先向左（赛道外侧）偏离，随后向右回切。16 路光电传感器首个误差信号 (Error) 峰值为**负值**。
    * **右半区 (Right Half)**：回程下行时，呈现为**两个连续的正向 S 形**。引导线首先向右（赛道外侧）偏离，随后向左回切。16 路传感器首个误差信号 (Error) 峰值为**正值**。
* **S 弯结束后的横移与转向逻辑 (Mirror Logic)**：
    * 两个 S 弯轨迹结束后的末端衔接了一段水平横移路径，用于对准后续的直道功能区。
    * **左半区 (Left)**：S 弯结束后路径向**右 (Right)** 水平横移，对齐直道后经过 **2.1 通信拱门**，随后执行一个 **90° 向左** 的直角转弯进入 1.3 方框区。
    * **右半区 (Right)**：S 弯结束后路径向**左 (Left)** 水平横移，对齐直道后经过 **2.1 通信拱门**，随后执行一个 **90° 向右** 的直角转弯进入 1.3 方框区。
* **通信节点 2.1 (Arch 1)**：
    * **位置**：位于 S 弯结束后的横移直道段上。
    * **特征**：设有高度为 50cm 的无线通信触发拱门。
* **程序校验**：此处极性序列必须与 `$g_MapSide$` 变量匹配。若 S 弯极性与后续横移/转弯方向出现逻辑冲突，程序应立即判定镜像识别失效。

### 4. 三方框顶角相连干扰区 (Corner-Connected Triple-Box: 2.1 -> 1.3)

该区域是巡线逻辑中最复杂的几何变向段，要求小车在三个交错的正方形边缘进行精确的轨迹切换。

* **物理构成**：由三个 50cm × 50cm 的正方形黑框组成。
* **拓扑结构特征**：
    * **底入顶出**：引导线从第一个方框的底边中心进入，从第三个方框的顶边中心退出。
    * **顶角相连**：方框之间通过顶点 (Corner) 直接衔接，而非主线贯穿。
    * **左右交错**：三个方框沿行进方向呈左右摆动分布。
* **详细行进轨迹逻辑 (以右侧半区为例)**：
    1. **入口 (Entry)**：引导线垂直连接至第一个正方形的 **底边中心点**。小车需在此 T 字路口执行 **90° 左转**，沿底边行驶至左下角，随后右转进入该方框的 **左侧边** 垂直直行。
    2. **顶点切换 1**：第一个方框的 **左上角** 直接连接第二个方框的 **右下角**。小车通过此连接顶点进入第二个方框，并立即转向沿其 **右侧边** 垂直直行。
    3. **顶点切换 2**：第二个方框的 **右上角** 直接连接第三个方框的 **左下角**。小车通过此顶点进入第三个方框，转向沿其 **左侧边** 垂直直行。
    4. **出口 (Exit)**：行驶至第三个方框顶端后，右转向进入顶边。引导线从第三个正方形的 **顶边中心点** 垂直向外延伸。小车需从中心点垂直退出（左转），恢复主线巡线。
* **详细行进轨迹逻辑 (以左侧半区为例)**：
    1. **入口 (Entry)**：主引导线垂直连接至第一个正方形的 **底边中心点**。小车需在此 T 字路口执行 **90° 右转**，沿底边行驶至右下角，随后执行 **左转** 进入该方框的 **右侧边** 垂直直行。
    2. **顶点切换 1**：第一个方框的 **右上角** 直接连接第二个方框的 **左下角**。小车通过此连接顶点跨入第二个方框，并立即转向沿其 **左侧边** 垂直直行。
    3. **顶点切换 2**：第二个方框的 **左上角** 直接连接第三个方框的 **右下角**。小车通过此顶点跨入第三个方框，并转向沿其 **右侧边** 垂直直行。
    4. **出口 (Exit)**：行驶至第三个方框顶端后，转向进入顶边。引导线从第三个正方形的 **顶边中心点** 垂直向外延伸。小车需从中心点垂直退出，恢复垂直主线巡线。
* **路口特征总结**：
    * 包含两个标准的 T 字型路口（入/出口）。
    * 包含多个 90° 直角转弯（方框内转角）。
    * 包含两个特殊的顶点衔接点，要求传感器在短时间内完成从一侧边缘到另一侧边缘的极性切换。

### 5. 四圆干扰区与雷达入口衔接 (Vertical Quad-Circle & Radar Entry: 1.3 -> 1.4)

该区域紧接在三方框区之后。小车从第三个方框顶边中心垂直退出后，需行驶约一段直道，随后进入由四个直径 50cm 的圆圈组成的复杂干扰区。

* **衔接逻辑 (From 1.3)**：
    * 小车从 1.3 节点的最后一个方框顶边中心点垂直驶出，进入垂直主引导线。
    * 行驶一段距离后，主线一侧出现垂直堆叠的四个圆圈（编号 1-4，从下至上）。

* **详细连接特征 (以右侧半区 Right Half 为例)**：
    * **主线方位**：主引导线位于圆圈组的 **右侧 (Right)**。圆圈组位于主线左侧（赛道内侧）。
    * **第一个圆 (Bottom Circle)**：主线通过一根水平横线向左延伸，连接至第一个圆的 **右侧边缘 (Right Edge)**。此点形成一个 T 字型干扰路口。
    * **中间圆 (Circles 2 & 3)**：第二个和第三个圆仅与上下圆相切，与右侧的主引导线**没有任何物理连接**，仅产生侧向视觉干扰。
    * **第四个圆 (Top Circle - 双侧连接)**：这是路径转折的关键点，具有双侧特征：
        1. **外侧连接**：主引导线继续向上延伸，通过第二根水平横线向左连接至第四个圆的 **右侧边缘 (Right Edge)**。
        2. **内侧出口**：在同一水平高度，一根新的引导线从第四个圆的 **左侧边缘 (Left Edge)** 向左延伸，直接通往雷达任务区。

* **镜像对称逻辑 (左侧半区 Left Half)**：
    * **主线方位**：主引导线位于圆圈组的 **左侧 (Left)**。圆圈组位于主线右侧（赛道内侧）。
    * **第一个圆**：主线向右延伸横线，连接至圆圈的 **左侧边缘 (Left Edge)**。
    * **第四个圆**：外侧连接圆的 **左侧边缘**；内侧出口从圆的 **右侧边缘** 向右延伸。

* **行进策略与环境上下文 (Transition to 1.4)**：
    * **路径选择**：小车需忽略所有向内侧（圆圈方向）延伸的支线，保持在垂直主线上直行。
    * **关键转折**：当检测到第四个圆上方的横线时，小车必须执行一个 **90° 向内侧（赛道中轴线方向）的直角转弯**。
    * **任务切换**：转弯后的水平线末端即为标记点 1.4。一旦传感器识别到 1.4 节点，必须立即停止巡线，切换为雷达扫描模式。

### 6. Task 3：雷达探测与避障任务箱 (Marker 3)

该区域是整个赛道中唯一的非固定巡线逻辑区，位于标记点 1.4 之后。

* **触发检测点 (Checkpoint 1.4)**：
    * [cite_start]**位置**：位于四圆干扰区结束后的水平引导线上 [cite: 67]。
    * [cite_start]**逻辑功能**：此处是切换传感器模式的关键点，屏幕必须在此处由巡线视觉切换为雷达频谱图（Waterfall 或 Spectrogram） [cite: 187]。
* [cite_start]**物理进入路径**：从水平引导线经过一个 90° 转弯进入垂直段，该直道段长度为 73.5cm [cite: 68]。
* **障碍任务箱 (Obstacle Box)**：
    * [cite_start]**外部尺寸**：一个矩形封闭区域，物理规格为 120cm（宽）x 70cm（高） [cite: 70, 71]。
* **内部障碍物 (Obstacle)**：
    * [cite_start]**材质与规格**：箱体内放置一块 50cm 宽、70cm 高的金属板障碍物 [cite: 136]。
    * [cite_start]**摆放逻辑**：金属板会随机放置在箱体内的左侧或右侧 [cite: 136]。
* **镜像对称性**：
    * **左侧半区**：小车需通过雷达扫描判定金属板在左或在右，并选择空余侧穿行。
    * **右侧半区**：任务逻辑与左侧完全镜像对称。
* [cite_start]**路径出口**：穿过任务箱后，路径重新汇入黑线，引向最终的 2.2 拱门与终点区域 [cite: 56]。

### 7. 雷达后干扰方块与终点衔接段 (Post-Radar -> 1.5)

该区域位于 Task 3 避障任务结束之后，包含一个非对称挂载的干扰方块和进入终点前的 U 型掉头结构。

* **终点前干扰方块 (Post-Radar Square)**：
    * **物理尺寸**：一个 50cm × 50cm 的正方形黑框。
    * **挂载特征**：引导黑线垂直向下延伸，干扰方块以**单侧挂载**的方式连接在黑线上（黑线即为方块的一条垂直边线）。
    * **镜像位置逻辑**：
        * **左侧半区 (Left Half)**：小车沿垂直线下行时，干扰方块挂载在引导线的 **左侧 (Left)**，即赛道外侧。
        * **右侧半区 (Right Half)**：小车沿垂直线下行时，干扰方块挂载在引导线的 **右侧 (Right)**，即赛道外侧。
* **U 型过渡弯 (U-Turn Transition)**：
    * **结构**：引导线在干扰方块的连接点处（垂直段最底端）进行 **180° 反向弯折**，向上方回绕。
    * **路径走向**：小车需驶过干扰方块的入口点，随即进入 180° 弯道转为上行。
* **末端转向与通信节点 (2.2)**：
    * **90° 转向逻辑**：小车沿 U 型弯上行一段距离后，需进行一个半圆转弯指向赛道中轴线的终点。
        * **左侧半区**：上行后向 **右 (Right)** 转弯。
        * **右侧半区**：上行后向 **左 (Left)** 转弯。
    * **通信拱门 (2.2)**：转弯后的水平直道上设有高度为 50cm 的第二个 LoRa 通信触发拱门。
* **终点区域 (Finish: 1.5)**：
    * **规格**：路径最终汇入赛道中心的 100cm × 50cm **红色**矩形感应区。
    * **动作**：识别到红色色块后，小车必须立即停止所有动力输出。

---

## 第三部分：Patio 赛道策略与控制建议

本部分结合 `L1b_Design-Tasks_An-Overview_2025-2026` 课件给出的 **Patio 赛道尺寸**（总长 800 cm、宽 500 cm、起终点 50×50 cm、直线与弯道、箱体区域以及 2.1/2.2 无线通信门、雷达箱体等），对现有 `微缩车模` 控制程序给出：

- **出发与起步路线识别策略**
- **基于光电阵列的赛道建模与速度自适应策略**
- **LoRa 无线通信的使用策略与标准库伪代码**
- **≥24 GHz 车载雷达在 Task 3 中的使用策略与标准库伪代码**
- **若干更高阶的改进建议**

---

### 一、出发与路线自动识别策略

**目标**：小车从 `Start` 绿框（50×50 cm）内任意合理摆放出发，**无需外部干预**，能够：

1. 自动找到并锁定 3 cm 宽黑线；
2. 自动判断当前是在 **左侧赛道还是右侧赛道**；
3. 在整个 8 m 赛道上稳定沿线前进，并按不同路段自适应调整速度和控制参数。

#### 1. 起步阶段的线搜索逻辑

- **光电阵列布局假设**：
  - 16 路传感器沿车宽度方向均匀铺开，跨越黑线宽度（3 cm）及左右空白区域；
  - 当前 `BlackPoint_Finder_Search()` 输出的 `precise_position` 可以视为 \[0, 15\] 范围的"线中心位置"。

- **起步搜索流程建议**：
  1. 上电后，小车在 `Start` 区域内先**缓慢直行**一小段（例如 10–20 cm，对应编码器脉冲或时间 0.5–1 s），同时持续调用 `BlackPoint_Finder_Search()`。
  2. 若在设定距离/时间内 `result_BlackPoint.found == 0`：
     - 认为当前未在黑线附近，可触发**小幅左右摇摆搜索**（例如左转 10° 再右转 20° 再回中），在搜索过程中一旦检测到连续若干次 `found == 1` 即认为成功锁定。
  3. 若连续 \(N\) 次（推荐 \(N ≥ 5\)）检测到 `found == 1`，且 `precise_position` 稳定在中间区域（比如 6–9 号传感器之间），则：
     - 设置 `star_car = 1`，允许 PID 速度环与位置环正式接管；
     - 将当前 `precise_position` 记为初始目标位置 `target_position`（通常接近中线 8.0）。

- **起步伪代码（逻辑级）**：

```c
state = STATE_START_SEARCH;

while (1) {
    switch (state) {
    case STATE_START_SEARCH:
        Motor_Enable();
        M3PWM_SetDutyCycle(SLOW_DUTY);   // 低速直行
        if (BlackPoint_Finder_Search(g_mux_adc_values, &result_BlackPoint) &&
            result_BlackPoint.found) {
            if (is_stable_on_line(result_BlackPoint.precise_position)) {
                PositionPID_SetTarget(&g_position_pid, result_BlackPoint.precise_position);
                star_car = 1;
                state = STATE_LINE_FOLLOW;
            }
        } else if (exceed_search_distance_or_time()) {
            state = STATE_SWING_SEARCH;   // 进入左右摇摆搜索
        }
        break;
    ...
    }
}
```

#### 2. 左右赛道自动识别

- 由于赛道两侧是**对称**的，只要赛道布置与课件一致，则从任意一端出发：
  - 视觉上，黑线整体相对于 Patio 全局坐标是镜像，但对小车本体坐标系而言，只要始终"在线中心附近"，控制逻辑无需区分"左赛道/右赛道"；
  - 真正需要区分的只是 **LoRa 触发的顺序和雷达箱体前的路径选择方向**。

- **识别方法建议**：
  1. 利用**开局第一个明显大弯道的方向**来区分：
     - 统计在前若干秒内 `position_get` 的偏差符号：如果总体偏右（`position_get > target`），说明黑线更多出现在右侧，对应一侧赛道；反之偏左对应另一侧。
  2. 一旦判定了"赛道类型"，设置一个枚举 `track_side = LEFT_TRACK / RIGHT_TRACK` 存入全局，用于后续：
     - 预判 LoRa 门所在的大致距离/时间窗口；
     - 预判 Task 3 时"箱体中自由侧"的默认方向（例如左赛道默认先扫描右侧箱壁，右赛道默认先扫描左侧箱壁），减少扫描时间。

---

### 二、赛道路段建模与速度自适应策略

结合 Patio 地图尺寸，可将一条赛道划分为若干**特征路段**：

1. **起跑直线**（约 70 cm）
2. **大 S 弯**（约 200 cm 高度内的曲线段）
3. **箱体/窄道组合区域**（多处 50 cm 宽的箱体/门、障碍区和 30–50 cm 的过渡直线）
4. **LoRa 门（2.1 / 2.2）附近的直线 + 箱体**
5. **Task 3 雷达箱体前的直线与箱体**
6. **终点前最后一段 S 弯和直线**

#### 1. 利用"黑线几何特征 + 速度反馈"识别路段

不依赖绝对位置传感器，仅靠当前已有传感器就可以做**弱建模**：

- 利用 `position_get` 的**一阶差分/二阶差分**判断弯道强度：
  - 弯道处 |`position_get - target_position`| 较大，且导数变化较快；
  - 直线处偏差绝对值和变化率都较小。
- 利用编码器积分估计**行驶距离**：
  - 将左右轮脉冲和转换为近似行驶距离，结合赛道标称长度对齐不同路段的"里程区间"；
  - 如：起跑 0–0.5 m，第一段 S 弯 0.5–1.5 m，LoRa 门前 2.5–3.0 m 等。

综合两者可以构建简化的路段状态机：

```c
typedef enum {
    SEG_START,
    SEG_S_CURVE_1,
    SEG_BOX_1,
    SEG_LORA_GATE_1,
    SEG_RADAR_BOX,
    SEG_S_CURVE_2,
    SEG_FINISH
} TrackSegment_t;
```

状态转移条件可以基于：

- 里程阈值（编码器积分）；
- 持续一段时间内的弯道强度（偏差方差）；
- 雷达/LoRa 特征事件（例如收到/成功发送 LoRa 包、检测到雷达箱体边缘等）。

#### 2. 不同路段的速度策略

基于当前 PID 结构，可以通过调节**期望速度 `i_speed` 上限 + PID 参数**实现：

- **起跑直线与长直线段**：
  - 目标速度较高，例如 \(v_{target} = 250–300\)（根据目前硬件测试确定上限）；
  - 位置环比例系数可以略减小，避免轻微噪声导致高频振荡。

- **大 S 弯和箱体入口**：
  - 根据当前位置偏差自动降低车速（现有代码已做：`i_speed = 230 - (fmin(fabs(current_position - 8),3)/3) * 100;`）。
  - 建议进一步引入**偏差变化率**（类似航向角变化），在偏差变化剧烈时主动降速。

- **窄箱体 / 雷达箱体内**：
  - 固定低速模式，例如 `i_speed_low = 150` 或更低；
  - 同时提高位置环 `Kp/Kd`，保证转弯与避障反应更快；
  - 利用 IMU 的角速度限制最大转向率，避免太急的差速导致甩尾。

- **终点前直线**：
  - 根据比赛策略可选择**略微提高速度**以冲线，但应增加"终点停车"逻辑：
    - 利用线末端的几何特征（如终点框内线段终止）+ 里程估计，在接近终点时降低速度并在 Finish 红框内缓慢停止。

#### 3. 多圆"误导区"最短路径（直上 + 只走顶半圆）

该区域（地图上靠近顶部、四个圆形堆叠的区域）确实是**光电阵列最容易被"分叉/环线"误导**的地方：如果单纯"取全局最黑点"跟随，车很容易钻进下面的圆环线路径。这里建议把该区域当作**有策略的特殊赛段**，明确分成两步：

- **步骤 A：直上（不进下方圆环）**
  目标是沿着主干竖直线一路到最上方，只允许小幅修正，避免"跳线"到下方圆环。
- **步骤 B：顶半圆（只绕最上方那一个圆的上半圈）**
  到顶部后执行一次**受约束的半圆转向**，把车从竖直方向"接"到上方水平连线（朝中间 Finish 方向）。

##### 3.1 左/右赛道的转向方向（镜像关系）

- **左侧赛道（左边 Start 出发）**：多圆在主干线**右侧**，顶半圆连接到上方水平线时需要整体**右转**（顺时针弧线）。
- **右侧赛道（右边 Start 出发）**：多圆在主干线**左侧**，顶半圆连接到上方水平线时需要整体**左转**（逆时针弧线）。

可用一个统一变量表示：

```c
turn_dir = (track_side == LEFT_TRACK) ? TURN_RIGHT : TURN_LEFT;
// TURN_RIGHT => 左轮更快、右轮更慢；TURN_LEFT 反之
```

##### 3.2 光电阵列如何"识别自己被误导了"

在多圆区，常见的误导现象是：阵列同一时刻"看到"不止一条黑线（例如主干线 + 圆环线），表现为：

- **多黑点同时出现**：归一化后很黑（接近 0）的通道数量明显增多；
- **左右两侧同时很黑**：左半阵列和右半阵列都出现很小的归一化值；
- **最黑点跳变**：`precise_position` 在很短时间内从靠中间/一侧跳到另一侧（跳变幅度大且频繁）。

建议在 `BlackPoint_Finder_Search()` 的结果之上加一个"分叉/环线检测"：

```c
bool is_confusing_zone = (black_count >= 3) || (left_has_black && right_has_black) || (jump_too_fast);
```

其中 `black_count` 可以用"归一化值 < 阈值"的通道数近似（阈值如 0.25），`jump_too_fast` 用最近 5~10 次 `precise_position` 的变化量/方差判断。

##### 3.3 步骤 A：直上阶段——"锁定主干线，不被下方圆环拉走"

直上阶段的核心不是追最黑，而是追"**与上一次主干位置一致**"：

- **规则 1（连续性优先）**：若检测到 `is_confusing_zone`，则不要切换到新的全局最黑点；改为选择**更靠近 `last_precise_position`** 的候选线（"就近跟随"）。
- **规则 2（远离圆环侧）**：在 `is_confusing_zone` 下，对 `precise_position` 施加一个小偏置，把车朝"远离圆环的一侧"压一点，以减少圆环线对阵列的覆盖概率。
  - 左侧赛道（圆在右）：偏置向左（减小 `precise_position`）
  - 右侧赛道（圆在左）：偏置向右（增大 `precise_position`）
- **速度规则**：直上阶段建议中低速（例如将 `i_speed` 上限限制在直线路段的 60%~75%），确保纠偏时不会甩进圆环。

##### 3.4 步骤 B：顶半圆阶段——"明确的左右轮差速 + 受约束的跟线"

顶半圆建议用"**约束转向**"来保证只走上半圈，不走下半圈：

- **控制目标**：在一小段时间/里程内保持"固定转向方向"，同时允许光电提供微调，但**禁止转向方向反复翻转**。
- **电机差速要点**（以恒定前进为主，不倒车）：
  - **右转（TURN_RIGHT）**：`left_speed > right_speed`（左轮快、右轮慢）
  - **左转（TURN_LEFT）**：`right_speed > left_speed`

结合现有输出形式（`left_output = base + corr; right_output = base - corr;`）可以直接记住：

- **`corr > 0` 产生右转**（左更快右更慢）
- **`corr < 0` 产生左转**

因此在顶半圆阶段可做"符号约束 + 限幅"：

```c
float corr = PositionPID_Calculate(&g_position_pid, current_position);

// 约束只允许指定方向转向（避免被下方圆环拉回）
if (turn_dir == TURN_RIGHT)  corr = fmaxf(corr, 0.0f);
else                         corr = fminf(corr, 0.0f);

// 限幅：避免打舵过猛
corr = clamp(corr, -CORR_MAX, +CORR_MAX);

left_output  = base_speed + corr;
right_output = base_speed - corr;
Motor_SetSpeedWithDirection(MOTOR_L, left_output);
Motor_SetSpeedWithDirection(MOTOR_R, right_output);
```

**顶半圆结束判据**建议用"里程 + 光电稳定性"双条件（比纯时间更稳）：

- **里程**：从进入顶半圆开始累计行驶距离达到预设（以地图尺寸估算后实测微调；该半圆直径标注约 20 cm，则半圆弧长约 \( \pi \times 10 \) cm ≈ 31 cm；考虑进出连接段，可把目标里程设为 40–60 cm 再标定）。
- **光电稳定**：当 `precise_position` 回到"上方水平主干线"的稳定区间（例如接近阵列中间），且 `is_confusing_zone == false` 持续 \(N\) 次，则退出顶半圆模式，回到正常循迹 PID。

---

### 三、光电阵列建模与控制策略（Task 1）

#### 1. 传感器几何建模

假设 16 路光电传感器沿车前端安装，中心间距约 \(d\) cm，总跨距 \(15d\)。黑线宽度为 3 cm，通常覆盖 1–3 个传感器。

- 定义**传感器坐标系**：
  - 设最左侧传感器坐标为 \(x_0 = -7.5d\)，编号依次为 0…15；
  - 第 \(i\) 个传感器坐标：\(x_i = -7.5d + i \cdot d\)；
  - `precise_position` 如为 7.8，则对应坐标 \(x = -7.5d + 7.8d = 0.3d\)。

- `PositionPID` 的 `target_position` 可直接设置为**车身几何中心对应的 `precise_position`**（约 7.5–8.0），从而将位置偏差解释为线相对车中心的横向偏移，控制目标是使该偏差为 0。

#### 2. 控制策略细化

- **多模态权重**：
  - 当前仅用光电阵列做横向误差；建议在中高速时引入 IMU 的**横摆角速度 `gz_rads` 限幅**：
    - 若 |`gz_rads`| 超过阈值（表示急转弯或打滑），则暂时降低速度环目标速度，并减弱位置环 `Kp`，防止过度修正。

- **丢线补偿**：
  - 利用 `last_precise_position`，在 `found == 0` 且时间未过长时，继续按"上一次线方向"进行小幅控制，并快速减速；
  - 同时利用 IMU 积分角度，在短时间内维持近似恒定曲率，增加重新找回黑线的概率。

---

### 四、LoRa 无线通信策略（Task 2）

课件要求：在标记点 2.1 与 2.2 的拱门下方，通过 **LoRa 协议**向上方接收端发送：

- 当前时间戳；
- 队号与队名；
- 自比赛开始以来的累计时间（分钟:秒）。

#### 1. 硬件与接口建议

- 选用常见 LoRa 模块（如基于 SX1276/78 的串口模块）接在 **STM32 的 USART（如 USART2）** 上，与现有 `Uart_Config` 模块兼容；
- 使用标准外设库 `stm32f10x_usart.h` 配置串口波特率（如 9600 / 115200），通过简单的**AT 命令**或自定义帧格式发送数据。

#### 2. LoRa 触发逻辑

- 拱门高度 50 cm，小车无高度传感器，因此建议通过**里程/时间窗口 + 黑线几何特征**估计接近 LoRa 门：
  - 在路段状态机中，为 `SEG_LORA_GATE_1` 和 `SEG_LORA_GATE_2` 分配大致里程区间（例如第一门约在 2–3 m 处，第二门约在 5–6 m 处，具体需实测微调）；
  - 在进入该区间时，开启"LoRa 预警窗口"：一旦检测到黑线前端进入**门框箱体的直线区域**（偏差较稳定，箱体两侧障碍线条特征），则在 0.5 s 内发送一次 LoRa 数据；
  - 为防止多次触发，可使用布尔标志 `lora_sent_gate1` / `lora_sent_gate2`。

#### 3. 基于标准库的伪代码示例

**串口与 LoRa 初始化**：

```c
void LoRa_Init(void)
{
    // 复用现有 Uart2_Init
    Uart2_Init(9600);  // 或模块要求的波特率

    // 若模块为 AT 指令型，可发送基本配置命令
    LoRa_SendAT("AT+MODE=LWOTAA\r\n");
    LoRa_SendAT("AT+DR=SF7BW125\r\n");
    // 其他根据模块手册配置的命令...
}

void LoRa_SendAT(const char *cmd)
{
    while (*cmd) {
        USART_SendData(USART2, (uint8_t)*cmd);
        while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
        cmd++;
    }
}
```

**在 gate 处发送数据帧**（假设采用简单的 ASCII 文本协议，上层由拱门接收端解析）：

```c
void LoRa_SendRaceInfo(uint8_t gate_id)
{
    char buf[64];
    uint32_t t_ms = get_race_time_ms(); // 比赛开始到现在的毫秒数
    uint32_t sec = t_ms / 1000;
    uint32_t min = sec / 60;
    sec = sec % 60;

    // TEAM_ID 和 TEAM_NAME 可在编译期通过宏定义
    sprintf(buf, "G%d,%s,%s,%02lu:%02lu\r\n",
            gate_id,
            TEAM_ID,
            TEAM_NAME,
            (unsigned long)min,
            (unsigned long)sec);

    const char *p = buf;
    while (*p) {
        USART_SendData(USART2, (uint8_t)*p);
        while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
        p++;
    }
}
```

**与赛道状态机结合**（在 SysTick 或主循环中）：

```c
if (current_segment == SEG_LORA_GATE_1 && !lora_sent_gate1) {
    if (under_gate_condition_met()) {  // 里程 & 几何特征满足
        LoRa_SendRaceInfo(1);
        lora_sent_gate1 = 1;
    }
}
```

---

### 五、≥24 GHz 雷达使用策略（Task 3）

任务要求：在 Task 3 的箱体中，障碍物随机放在左或右一侧，小车必须在进入箱体前用**≥24 GHz 汽车雷达**扫描并判断**哪一侧有障碍**，然后选择另一侧通行。

#### 1. 雷达安装与指向建议

- 使用提供的 24 GHz FMCW 雷达模块（如 CYCPLUS 模块），前向安装，略向下倾斜，波束可覆盖前方箱体左右两侧；
- 为了区分左右障碍，可选两种方案：
  1. **单雷达 + 机械或电子扫描**：利用雷达较宽的波束，通过在左右两侧定义**兴趣区域 (ROI)**，分别读取距离/反射强度；
  2. **双雷达**：左右各一块，分别负责对应一侧，软件逻辑更简单，但成本更高。

在预算允许且布线方便的情况下，**推荐双雷达**：逻辑清晰、可靠性更高。

#### 2. 雷达箱体前的行为策略

1. 通过里程估计接近 Task 3 箱体入口（例如 5–6 m 范围内）；
2. 在距离箱体入口约 30–50 cm 处减速进入"雷达扫描模式"：
   - 固定低速直行，保持在黑线中心；
   - 连续 \(N\) 次读取左右雷达的目标距离（或反射强度）；
3. 判断哪一侧存在障碍：
   - 例如：左雷达最近目标距离 \(d_L < 60\) cm 且回波强度高，而右雷达无明显目标或距离 \(d_R > 80\) cm，则认为**障碍在左侧**；
4. 选择**相反侧**作为通道：在进入箱体时偏向无障碍一侧，并在箱体内部保持较小但固定的侧向偏移（类似"沿箱壁走"）。

#### 3. 基于标准库的伪代码示例

假设雷达模块通过 UART 或 SPI 提供**距离数据**，以 UART 为例（类似 LoRa）：

```c
typedef struct {
    uint16_t distance_cm;
    uint8_t  valid;
} RadarMeasure_t;

RadarMeasure_t radar_left, radar_right;

void Radar_Init(void)
{
    // 假设左雷达用 USART3，右雷达用 USART1
    USART_InitTypeDef us;
    // 参考 stm32f10x_usart 标准库配置...
}

void Radar_ReadLeft(RadarMeasure_t *m)
{
    // 从串口缓冲区解析一帧雷达数据，得到最近目标距离
    // 这里用伪代码，实际需参考具体模块协议
    if (frame_ok) {
        m->distance_cm = parsed_distance;
        m->valid = 1;
    } else {
        m->valid = 0;
    }
}

void Radar_ReadRight(RadarMeasure_t *m)
{
    // 同上，读取右雷达
}
```

**扫描与决策逻辑**：

```c
typedef enum {
    RADAR_UNKNOWN = 0,
    RADAR_OBSTACLE_LEFT,
    RADAR_OBSTACLE_RIGHT
} RadarDecision_t;

RadarDecision_t Radar_ScanBox(void)
{
    uint8_t i;
    uint16_t dL_min = 1000, dR_min = 1000;

    for (i = 0; i < 10; i++) {  // 连续读取多次，增强可靠性
        Radar_ReadLeft(&radar_left);
        Radar_ReadRight(&radar_right);
        if (radar_left.valid && radar_left.distance_cm < dL_min) {
            dL_min = radar_left.distance_cm;
        }
        if (radar_right.valid && radar_right.distance_cm < dR_min) {
            dR_min = radar_right.distance_cm;
        }
        Delay_ms(20);
    }

    if (dL_min < OBSTACLE_THRESHOLD_CM && dR_min > FREE_THRESHOLD_CM) {
        return RADAR_OBSTACLE_LEFT;
    } else if (dR_min < OBSTACLE_THRESHOLD_CM && dL_min > FREE_THRESHOLD_CM) {
        return RADAR_OBSTACLE_RIGHT;
    } else {
        return RADAR_UNKNOWN;  // 无法可靠判断，保守策略
    }
}
```

**与车体路径控制结合**：

```c
void Handle_RadarBox(void)
{
    RadarDecision_t dec = Radar_ScanBox();

    switch (dec) {
    case RADAR_OBSTACLE_LEFT:
        // 障碍在左侧 -> 走右侧
        PositionPID_SetTarget(&g_position_pid, RIGHT_WALL_FOLLOW_POS);
        break;
    case RADAR_OBSTACLE_RIGHT:
        // 障碍在右侧 -> 走左侧
        PositionPID_SetTarget(&g_position_pid, LEFT_WALL_FOLLOW_POS);
        break;
    case RADAR_UNKNOWN:
    default:
        // 保守策略：根据默认赛道方向选择一侧并进一步减速
        if (track_side == LEFT_TRACK)
            PositionPID_SetTarget(&g_position_pid, RIGHT_WALL_FOLLOW_POS);
        else
            PositionPID_SetTarget(&g_position_pid, LEFT_WALL_FOLLOW_POS);
        i_speed = VERY_SLOW_SPEED;
        break;
    }
}
```

其中 `LEFT_WALL_FOLLOW_POS` / `RIGHT_WALL_FOLLOW_POS` 分别是沿箱体左壁/右壁行驶时，对应的 `precise_position` 目标值（可通过实验标定，例如靠近左侧时线在第 5–6 号传感器附近）。

---

### 六、更高阶的改进建议

1. **根据赛道模型做"速度规划曲线"而不是简单的 if-else**
   - 利用里程估计，将赛道分段后为每一段预设目标速度曲线（类似赛车的"赛道地图"），再叠加当前偏差/IMU 的反馈动态缩放；
   - 这比单纯根据瞬时偏差降速更平滑、更可预测。

2. **使用模型预测控制（MPC）或前瞻控制**
   - 在现有双环 PID 的基础上加入**预瞻距离**概念，例如根据当前偏差和其导数预测若干采样周期后的偏差，将预测值作为控制输入；
   - 可以大幅提升高速弯道下的稳定性，减小超调。

3. **联合雷达与光电实现"障碍感知 + 轨迹重规划"**
   - 当前 Task 3 只在箱体中使用雷达，实际上可以在整条赛道上用雷达检测前方大障碍（误入他队车道、工作人员进入赛道等），并执行紧急刹车或绕行；
   - 通过将雷达距离作为额外约束输入到速度规划中（例如前车距 < 某阈值时自动限速）。

4. **数据记录与离线分析**
   - 使用串口或外接存储（如 SD 卡）定期记录关键数据：`position_get`、`speed_left/right`、`gz_rads`、`i_speed`、赛段 ID 等；
   - 赛后在 PC 上进行可视化分析（Matlab/Python），针对**每个弯道和箱体**寻找最佳 PID 参数与速度上限。

5. **参数自标定与赛前自检流程**
   - 在比赛前加入一键"自检与标定"：
     - 在固定白纸/黑线环境下自动扫描 16 路光电，重新估计 `min/max` 与阈值；
     - 简单直线加减速测试，自动测量最大安全加速度与制动距离；
   - 将结果保存到 Flash 或 EEPROM 中，使得在不同光照与地板条件下都能快速适应。

6. **软件结构化与模式管理**
   - 将整车运行抽象为有限状态机（FSM），例如 `IDLE / START_SEARCH / LINE_FOLLOW / LORA_GATE / RADAR_SCAN / BOX_PASS / FINISH / EMERGENCY_STOP`；
   - 使得不同任务（Task 1/2/3）在代码结构上清晰解耦，也方便调试与评分展示。

---

以上建议与伪代码均基于当前 `微缩车模` 工程的**标准外设库结构**和已有模块（电机、光电阵列、IMU、PID 控制等）设计，实际实现时可逐步迭代：先搭建状态机与里程估计框架，再逐段调参与验证，以尽量在有限时间内让小车在 Patio 赛道上实现**稳定、自主、快速**的整体表现。

---

## 第四部分：GitHub 仓库协作指南

### 一、环境准备 (Environment Setup)

在开始编写微缩车模代码之前，请团队所有成员务必严格按照以下步骤配置本地开发环境：

#### 1. 安装 Git 版本控制引擎
* **下载地址：** 前往 [Git 官方网站](https://git-scm.com/) 下载对应操作系统的最新 64 位版本。
* **安装注意：** 安装过程中一路点击 "Next" 即可。在选择默认编辑器时，建议在下拉菜单中选择 **"Use Visual Studio Code as Git's default editor"**。请确保 Git 被成功添加到了系统的环境变量 (PATH) 中。

#### 2. 安装 VS Code (代码编写与版本管理主战平台)
* **下载地址：** 前往 [Visual Studio Code 官网](https://code.visualstudio.com/) 下载。
* **必装插件 (Extensions)：** 打开 VS Code，在左侧扩展市场安装以下 3 个官方插件：
    * **C/C++ (Microsoft)：** 提供底层 C 语言的语法高亮、代码跳转与智能提示 (IntelliSense)。
    * **GitLens — Git supercharged：** 工程级查错神器，可在代码行末尾直接显示该行代码是谁在何时修改的（精准责任追踪）。
    * **Makefile Tools (Microsoft)：** 由于已在云端配置了 GCC 自动化测试，此插件可解析根目录下的 Makefile，自动补全库路径，消除本地代码的红色波浪线误报。

#### 3. 安装 Keil MDK (物理编译与烧录工具)
* **版本要求：** 建议使用 Keil uVision5 版本。
* **芯片包依赖 (Device Family Pack)：** 确保已通过 Pack Installer 安装了 `Keil.STM32F1xx_DFP` 支持包，否则无法打开和编译本工程的 `.uvprojx` 文件。

---

### 二、初次获取代码（Clone）

此操作每台电脑仅需执行一次。这会将云端完整的 Git 数据库及所有历史快照物理下载到你的本地硬盘。

> ⚠️ **前置极其重要条件：** 本仓库为私有仓库 (Private Repository)。在执行以下步骤前，请务必先登录你的 GitHub 账号，并在网页右上角的通知 (Notifications) 铃铛图标或绑定的邮箱中，点击**接受 (Accept Invitation)** 仓库协作邀请。未接受邀请直接克隆将报 `404 Not Found` 权限拒绝错误。

**详细操作步骤：**

1.  **启动 VS Code：** 打开一个干净的 VS Code 界面。
2.  **调出命令面板：** 按下键盘快捷键 `Ctrl + Shift + P` (Windows) 或 `Cmd + Shift + P` (Mac)。
3.  **输入克隆指令：** 在弹出的顶部搜索框中，输入 `Git: Clone`（或中文界面下的 `Git: 克隆`），并点击下拉列表选中该行。
4.  **输入仓库地址：** 在提示框中精准粘贴本团队的云端仓库 URL： `https://github.com/Bigrreedd/TDPS_MiniCar_Control.git` 粘贴后按回车键 (Enter)。
5.  **GitHub 身份验证 (Authentication)：** 如果你是第一次在电脑上使用 Git 连接 GitHub，VS Code 或系统弹窗会要求你进行身份验证。请选择 **"Sign in with your browser"** (使用浏览器登录)，并在弹出的网页中点击 **"Authorize GitCredentialManager"** 完成授权绑定。
6.  **选择本地存储路径 (红线警告)：** 在弹出的文件资源管理器中，选择一个本地文件夹来存放整个车模工程。
    * ⛔ **绝对禁止包含中文或空格的路径！** （例如：`D:\我的大学\大三下\微缩车模` 是严重违规的，这会导致 Keil 底层链接器彻底罢工报错）。
    * ✅ **标准规范路径示例：** `D:\Workspace\UESTCHN3018\TDPS_Project`。
7.  **打开工程工作区：** 等待右下角进度条跑完。下载完成后，VS Code 右下角会弹出提示，直接点击 **"Open"** (打开)。此时，左侧资源管理器会显示所有 `.c`, `.h` 及 `.uvprojx` 文件，代码接入完成！

---

### 三、日常协作核心工作流 (The Core Workflow)

为了保证底层 C 语言代码数据库的线性一致性，每次开始编写代码或准备共享代码时，全队 5 人必须严格遵循以下单向数据流操作：

#### 1. 编写前必做：拉取同步 (Pull)
* **技术本质：** 执行 `git fetch`（从云端下载最新的 Blob、Tree 和 Commit 对象）并触发 `git merge`（将本地 `HEAD` 指针向前推进或进行合并）。
* **工程目的：** 强制本地工作区与 GitHub 远端数据库对齐，从物理层面上最大程度降低后续的合并冲突概率。
* **操作指令：** 在 VS Code 左侧的 **源代码管理 (Source Control)** 面板中，点击面板顶部标题栏右侧的 `...` (更多操作) -> 选择 **拉取 (Pull)**。
* 🔴 **红线警告：** 每天到达实验室打开电脑的第一件事，必须是执行 Pull 操作。

#### 2. 本地物理修改 (Modify)
* 在 Keil MDK 或 VS Code 中正常修改 `.c`, `.h` 或 `.uvprojx` 文件。
* **状态同步：** 修改完成后，务必在编辑器中按 `Ctrl + S` 保存物理文件，VS Code 底层的 Git 引擎才会重新计算文件的 SHA-1 哈希值并识别到 **(Modified)** 状态。

#### 3. 暂存与提交 (Stage & Commit)
* **技术本质：** 将工作区的变更写入 `.git/index`（暂存区），随后生成永久的 Commit 对象（快照）并推进本地指针。
* **操作步骤：**
    1.  **审查更改：** 在 **更改 (Changes)** 列表中，点击文件名查看差异视图 (Diff)，确认没有误删关键代码。
    2.  **暂存文件：** 点击待同步文件右侧的 `+` 号（底层等同于 `git add <file>`）。
    3.  **规范化提交：** 在顶部的消息框中输入符合语义化版本规范的提交说明。
    4.  **生成快照：** 点击 **提交 (Commit)**。
* **团队 Commit 命名规范 (Semantic Commits)：** 必须采用 `<类型>: <具体修改的模块及技术原因>` 的格式：
    * `feat:` 新增功能（例：`feat: 增加 LoRa 串口通信初始化函数`）
    * `fix:` 修复逻辑或语法错误（例：`fix: 修正光电阵列归一化分母为0的致命错误`）
    * `docs:` 文档或注释修改（例：`docs: 更新电机 PWM 占空比计算公式说明`）
    * `refactor:` 代码重构，不改变行为（例：`refactor: 提取 PID 计算部分为独立函数`）

#### 4. 推送至云端与 CI 触发 (Push & CI Check)
* **技术本质：** 将本地新增的 Commit 对象打包，通过 HTTPS/SSH 协议传输至 GitHub 服务器，并更新远端 `origin` 指针。
* **操作步骤：** 点击面板中的蓝色按钮 **同步更改 (Sync Changes)**。
* ⚠️ **CI 自动化安检 (Continuous Integration)：** 代码推送后，GitHub Actions 会在云端 Linux 服务器上自动拉取并执行 `make all`。请前往 GitHub 仓库主页查看提交记录旁边的状态指示灯：
    * **绿勾 ✅：** 编译通过，代码安全。
    * **红叉 ❌：** 存在语法错误或缺少头文件。提交者必须立刻在本地修复错误并重新提交，其他队员看到红叉时绝对禁止执行 Pull 操作！

---

### 四、团队分支与防冲突策略 (Branching & Conflict Resolution)

5 人同时在单片机工程中修改代码（尤其是 `main.c` 和 `stm32f10x_it.c` 中断文件）极易造成底层数据覆写。本工程严格执行"主干保护与功能隔离"策略：

#### 1. 绝对的功能隔离 (Feature Branching)
任何独立模块（如 Task1 循迹、Task3 雷达扫描）的开发，严禁直接在 `main` 分支上进行。`main` 分支必须永远保持 100% 能够通过编译且能在赛道上稳定运行的状态。
* **创建工作指针 (Create Branch)：** 点击 VS Code 左下角的 `main` 按钮 -> 选择 **创建新分支**。命名规范：`姓名缩写/功能名`（例：`ZJ/feature-radar`, `LHX/lora-task`）。
* **云端备份 (Publish Branch)：** 点击左下角的云朵图标，将此指针同步至 GitHub。
* **合并准入 (Merge to Main)：**
    1.  在实车硬件上测试无逻辑 Bug。
    2.  确认云端 CI 编译状态为 **绿勾 ✅**。
    3.  切换回 `main` 分支，按 `Ctrl+Shift+P` 输入 `Git: Merge Branch`，选择你的功能分支执行合并，并最终 Push 到云端。

#### 2. 解决合并冲突的底层方法 (Resolve Merge Conflicts)
当两名队员修改了同一个 `.c` 文件的同一行代码并尝试合并时，Git 会停止合并动作，在文件中插入标准的冲突定界符。

**冲突表现示例 (如 PID 调参冲突)：**
```c
<<<<<<< HEAD (当前更改 - 你本地 main 分支的代码)
float Kp = 12.5;  // 你认为小车应该用这个参数
=======
float Kp = 15.0;  // 队友推送的新参数
>>>>>>> ZJ/feature-radar (传入的更改 - 试图合并进来的代码)
```

**解决方法：**
1. 在 VS Code 中打开冲突文件，冲突区域会以**红色高亮**显示。
2. 点击顶部弹出的 **"Accept Current Change"** (保留你的) 或 **"Accept Incoming Change"** (采纳队友的) 按钮，或手动编辑合并后的内容。
3. 删除所有 `<<<<<<<`、`=======`、`>>>>>>>` 标记行。
4. 保存文件 → 重新 Stage & Commit → Push。

---

## 第五部分：中期汇报资料（Mid-term Presentation Slides）

> 来源：`midterm_slides/PreSlides.html`（2026/4/21 汇报用）

### 团队信息

- **课程**：UESTCHN3018 · Team Design Project & Skills
- **团队**：G2 · Team 20
- **机构**：Glasgow College Hainan, UESTC
- **阶段**：Mid-term Presentation（中期汇报）

### 团队成员与分工

| 成员 | 英文名 | 角色 | 职责 |
|------|--------|------|------|
| 黄竞逸 | Huang Jingyi | **Project Manager** | 整体系统架构、任务分配与协调、集成规划、动力总成决策 |
| 白子鹤 | Bai Zihe | **RF & PCB Engineer** | LoRa 集成、24 GHz 雷达集成、雷达与通信板设计、ESP32-S3 上位板硬件 |
| 卞明祥 | Bian Mingxiang | **Mechanical Engineer** | 底盘与结构设计、紧凑封装、安装布局、机械优化 |
| 陈可 | Chen Ke | **Documentation** | 文档协调、进度管理、演示支持、规划记录 |
| 雷皓翔 | Lei Haoxiang | **HW-SW Engineer** | 板级启动与测试、固件-硬件联合调试、系统级集成支持、实际验证 |

### 系统架构：两级控制结构

系统采用 **Master–Slave** 两级控制架构：

- **主控板 (Master)**：STM32F103C8T6
  - 处理所有光电/编码器信号
  - 运行双环 PID + 状态机
  - 管理 IMU、OLED、蓝牙
  - 中央电源分配枢纽

- **从板 (Slave)**：ESP32-S3
  - 仅负责 LoRa + 24 GHz 雷达
  - 不接入其他模块
  - 通过 UART 发送障碍物数据包
  - 不参与驱动控制环路

- **板间通信**：UART（障碍物数据包），Master 向 Slave 提供 5V 供电

### 五层车辆架构 (Five-Layer Vehicle Stack)

| 层级 | 名称 | 组成 |
|------|------|------|
| Layer 01 | **Sensing（感知）** | 24 GHz 雷达 HLK-LD2451、16 通道光电阵列（74HC4067 多路复用、8-pin FFC）、IMU LSM6DSRTR (SPI)、磁编码器 L/R |
| Layer 02 | **Actuation（执行）** | 吸风风扇（高速 NdFeB）、双驱动电机（NdFeB + 内置磁铁） |
| Layer 03 | **Control（控制）** | 主控 STM32F103C8T6、从板 ESP32-S3、板间 UART 链路 |
| Layer 04 | **Power（电源）** | Tattu 3S 11.1V 850mAh LiPo、XT30 接口、5V 轨由 Master → Slave |
| Layer 05 | **Structure（结构）** | 微缩底盘（加大吸附面积）、尼龙纸裙边（气密性）、方形低离地间隙底板 |

### 感知层详解 (Layer 01 · Sensing)

**24 GHz 毫米波雷达：**
- 模块：HLK-LD2451
- 频段：24 GHz FMCW
- 用途：前方障碍物探测（盲盒区域专用扫描）
- 连接：挂载在 ESP32-S3 从板上

**16 通道光电阵列：**
- 形状：锚形（Anchor form）前端展宽
- 多路复用器：74HC4067
- 连接线缆：0.5mm 8-pin FFC
- 特点：高偏差灵敏度、弯道中更早检测丢线

**IMU + 编码器：**
- IMU：LSM6DSRTR，SPI 接口
- 编码器：磁编码器（左/右轮各一个）
- 输出：轮速 ω + 偏航角
- 用途：PID 闭环反馈、航向漂移抑制

### 吸风风扇 (Layer 02 · Suction Fan)

- **原理**：垂直 NdFeB 风扇安装在两个驱动电机之间，抽气产生底盘下方负压
- **效果**：额外下压力 → 更高轮胎摩擦力 → 弯道中抗离心打滑
- **关键公式**：
  - F_down = ΔP · A（下压力 = 气压差 × 裙边面积）
  - f_max = μ · (mg + F_down)（最大摩擦力随下压力增长）
  - v_corner ≤ √(μ g r (1 + F_down/mg))（弯道极限速度与吸附力成正比）
- **结果**：更高弯道速度、无打滑、更好的功率-速度转化

### 双驱动电机 (Layer 02 · Drive Motors)

- **类型**：竞赛级 NdFeB 高速电机
- **特点**：轴承 + 径向磁铁嵌入安装柱、拇指大小磁编码器板直接插接、差速驱动转向
- **与标准 520 电机对比**：

| 参数 | 我们的配置 | 标准 520 |
|------|-----------|----------|
| 磁铁 | NdFeB | Ferrite |
| 电压 | 11.1V (3S) | 7.4V (2S) |
| 空载转速 | ~30,000 RPM | ~16,000 RPM |
| 峰值扭矩 | ~2.6× 原厂 | 基准 |
| 峰值功率 | ~120W/个 | ~45W/个 |
| 编码器 | 集成磁编码 | 外挂 |

- 功率密度 >2×，转速 1.9×，集成 AB 正交编码

### 电源与结构 (Layer 04 & 05)

**电源：**
- 品牌：Tattu
- 电芯：3S LiPo
- 电压：11.1V 标称
- 容量：850mAh
- 接口：XT30
- 路径：电池 → Master 板 → Slave 板 (5V)

**结构：**
- 微缩底盘 → 扩大吸附影响面积
- 方形底板 · 低离地间隙
- 底部尼龙纸裙边 → 气密性
- 垂直风扇位于两电机之间
- 铜柱堆叠：Master 板 ↑ Slave 板

### Task 1：循迹 (Line Following)

**需求**：识别赛道中心线、高速稳定行驶、处理直线与变曲率弯道

**方案**：
- 亚像素加权质心插值
- IMU Δψ 抑制航向漂移
- 双环 PID：外环位置、内环速度

**控制链路**：
1. 16 通道 → 多路复用 → ADC
2. 加权质心 → x_err（横向偏差）
3. IMU 融合航向
4. 位置环 PID
5. 速度环 PID（编码器反馈）
6. 差速 PWM 输出

**关键公式**：`x_err = Σ(w_i · s_i) / Σ(s_i) - x_centre`（加权质心插值 → 横向线位置误差）

### Task 2：无线通信 (LoRa)

**需求**：通过拱门区域、实时传输队伍+状态数据

**方案**：
- LoRa 模块位于 ESP32-S3 从板
- 在拱门处 ESP32 发送 AT 指令
- 数据包内容：{ ID · 队名 · 累计时间 }
- 通信功能在上位板隔离，不影响驱动环路

### Task 3：雷达避障 + 最短路径

**雷达避障流程**：
1. 靠近箱体时减速
2. ESP32 轮询 LD2451 雷达
3. 判定障碍物在 LEFT / RIGHT
4. 通过 UART 发送给主控 STM32
5. 主控选择空闲车道通行

**最短路径规划**：
- 利用 IMU 检测首次 U 型弯方向
- 偏航角判定起始赛道侧（左/右）
- 预选镜像策略
- 轻量级方案，无需全局求解器

### 硬件进展（PCB 模块）

中期汇报时已完成以下 5 块 PCB 的制造：

| PCB 模块 | 描述 | 核心器件 |
|----------|------|----------|
| Motor Driver Board（主控板） | 电机驱动板 | STM32F103C8T6、风扇开孔 |
| 16-ch Photo Array | 16 通道光电阵列板 | 74HC4067、锚形布局 |
| Radar / LoRa（从板） | 雷达与通信板 | ESP32-S3、24 GHz + LoRa |
| Left Encoder | 左编码器板 | 磁编码、拇指大小 |
| Right Encoder | 右编码器板 | 磁编码、拇指大小 |

**已完成状态**：4 块 PCB 制造完成、风扇已安装、电池已对接、所有硬件已物理集成。

---

## 附录：文件清单

| 文件 | 说明 |
|------|------|
| `README.md` | 控制程序说明（代码结构与模块详解） |
| `map.md` | Patio 赛道地图详解（几何尺寸、节点特征、镜像逻辑） |
| `advice.md` | 赛道策略与控制建议（循迹策略、LoRa 通信、雷达避障、改进建议） |
| `Github仓库.md` | 团队协作指南（Git 工作流、分支策略、CI 编译检查） |
| `L1b_Design-Tasks_An-Overview_2025-2026 Final.pdf` | 课程官方设计任务概述（PDF） |
| `midterm_slides/PreSlides.html` | 中期汇报演示文稿（HTML 格式，含系统架构、硬件展示、任务方案） |
| `midterm_slides/*.png` | PCB 实物照片与赛道路径图 |
| `pdf2md.bat` | PDF 转 Markdown 工具脚本 |
| `TDPS_完整背景资料.md` | 本合并文档（包含以上所有内容） |

---

## 开发日志：控制程序优化记录

### 2026-03-12 代码审查与重构

#### 一、SysTick 中断瘦身

**问题**：`SysTick_Handler` 中执行 16 通道 ADC 轮询、黑线识别、PID 计算，中断执行时间过长（~170µs ADC + 计算），导致其他中断被阻塞。

**方案**：将 ADC 采样、`BlackPoint_Finder_Search`、`PID_Control_Update` 全部移到主循环，中断仅设置 `g_control_tick = 1` 标志位。主循环检测标志位后执行耗时操作。

**涉及文件**：`stm32f10x_it.c`、`main.c`

#### 二、赛道状态机实现（Path.c）

**新增文件**：`Path.h` + `Path.c`

实现了完整的 10 段赛道状态机：

| 段枚举 | 功能 | 退出条件 |
|--------|------|---------|
| `SEG_START_SEARCH` | 起步摇摆寻线 | 稳线 ≥ 10 tick 或距离 > 30cm |
| `SEG_START_STRAIGHT` | 起步直行稳定 | 距离 > 20cm |
| `SEG_LINE_FOLLOW` | 正常循迹 | 里程进入特殊路段区间 + 弯道/丢线触发 |
| `SEG_U_TURN` | 180° U 弯 | 段内曾丢线 + 重新找回线，或距离 > 100cm 兜底 |
| `SEG_S_CURVE` | 双重 S 弯 | 弯道消失 + 稳线，或距离 > 150cm |
| `SEG_BOX_1` | 三方框顶角区 | 稳线 + 非弯道，或距离 > 120cm |
| `SEG_QUAD_CIRCLES` | 四圆干扰区 | 距离 > 120cm + 稳线，或距离 > 200cm |
| `SEG_RADAR_APPROACH` | 雷达箱体减速 | 总里程 > 750cm |
| `SEG_FINISH_APPROACH` | 终点前 U 弯+直线 | 段距离 > 60cm |
| `SEG_FINISH` | 终点停车 | — |

**里程估计**：`Path_UpdateOdometer(left_delta, right_delta)` 在 `PID_Control_Update` 中每 2ms 调用一次，累积 `total_dist_cm`（编码器脉冲 → cm 转换，比例 `ENCODER_TICKS_PER_CM` 需实测标定）。

**弯道检测**：`CalcCurveStrength()` 使用 `position_get` 的 8 样本滑动窗口方差近似弯道强度，阈值 2000。

#### 三、关键 Bug 修复

1. **U 弯退出条件不可靠** — 原条件 `line_lost_count > 30 && line_stable_count >= 10` 因两个计数器互斥递减，同时满足的窗口极窄（~几 tick）。改为 `seg_had_line_loss` 段内标记：一旦 `line_lost_count > LINE_LOST_THRESHOLD` 就置 1，段转移时清零。退出条件变为 `seg_had_line_loss && line_stable_count >= 10`。

2. **计数器溢出** — `line_stable_count`/`line_lost_count` 为 `uint16_t`，500Hz 下 ~131 秒溢出。加 `COUNT_SAT = 1000` 饱和上限。

3. **K2 停车不重置 `lose_time`** — 再次发车时可能直接触发 500 tick 超时保护导致立即停车。已补 `lose_time = 0`。

#### 四、代码质量改进

- `Path.c` 中 3 处函数体内 `extern` 声明移至文件顶部
- 删除未使用的 `WHEEL_BASE_CM` 宏定义
- 删除 `BlackPoint_Finder.c` 中未使用的 `sum` 变量及累加

#### 五、编译结果

```
   text    data     bss     dec     hex  filename
  18128     136    2064   20328    4f68  TDPS_MiniCar.elf
```

0 error, 0 warning。Flash 占用约 18KB（STM32F103C8T6 共 64KB），RAM 约 2KB（共 20KB）。

#### 六、待标定参数

| 参数 | 当前值 | 说明 |
|------|--------|------|
| `ENCODER_TICKS_PER_CM` | 6.0f | 编码器脉冲/cm，需实车标定 |
| `DIST_U_TURN_ZONE` ~ `DIST_FINISH` | 30~750cm | 各段里程阈值，需根据赛道实测 |
| `SPEED_SEARCH` ~ `SPEED_FINISH` | 150~400 | 各段目标速度（占空比/1000），需调参 |
| `curve_strength` 阈值 | 2000.0f | 弯道检测方差阈值，需实测 |
| PID 参数 | 已有值 | 速度环/位置环 PID 增益，需实车调试 |