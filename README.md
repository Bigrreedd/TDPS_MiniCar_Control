微缩车模控制程序说明

本目录为基于 STM32F103 的微缩车模控制工程（Keil MDK）。代码以标准外设库为基础，实现循迹黑线识别、双闭环速度位置控制、姿态与陀螺仪补偿等整车控制逻辑。以下按普通说明文档书写，便于复制到 Word 使用。


一、目录结构概览

根目录包含 Keil 工程文件 Project.uvprojx、Project.uvoptx、Project.uvguix 等，以及 JLinkSettings.ini、JLinkLog.txt、EventRecorderStub.scvd、keilkill.bat 等仿真与清理脚本。

Start 目录为启动与内核相关：startup_stm32f10x 系列，system_stm32f10x，core_cm3 等。

System 目录为 Delay 毫秒与微秒延时。

Library 目录为 STM32 标准外设库，GPIO、ADC、TIM、USART 等。

User 目录为用户应用与控制逻辑，各 c 与 h 文件为相对独立功能模块，详见下节。

Objects、Listings、DebugConfig 为编译生成与调试配置，一般不必手工修改。


二、控制模块说明（User 目录）

总体说明：用户控制逻辑集中在 User 目录，由 main.c 作为入口，stm32f10x_it.c 中的 SysTick_Handler 作为核心周期调度。各模块通过全局变量与函数调用协同，形成传感器采样、线路识别、双环 PID、电机驱动的闭环。

main.c 主程序入口：初始化 RGB、OLED、IMU（LSM6DSR）、M3PWM、电机（Motor_ctr）、编码器（ABEncoder）、多路 ADC（ADC_get）、按键（Key_Scan）、串口（Uart_Config）、循迹（BlackPoint_Finder）、PID（PID_Controller）等；配置 SysTick；主循环中读取按键、设置 RGB、启停电机与 M3PWM 占空比、通过 star_car 控制是否运行；车载电压 BDI_V 由 g_mux_adc_values 下标 16 的 ADC 结果换算；OLED 遥测界面由 TelemetryScreen 模块统一刷新（见 Software 目录下 log 记录）。

stm32f10x_it.c 中断与周期调度：SysTick_Handler 中周期性读取 IMU、积分 add_angle、串口看门狗、编码器速度、MuxADC 采样、BlackPoint_Finder_Search、PID_Control_Update 等。

Motor_ctr.c 与 Motor_ctr.h：两路直流电机使能、方向、TIM1 PWM，占空比参数为零到一万，与 M3PWM 模块的 TIM2 路不同。

PID_Controller.c 与 PID_Controller.h：速度环与位置环 PID，输出经 Motor_SetSpeedWithDirection 下发到电机。

BlackPoint_Finder.c 与 BlackPoint_Finder.h：十六路光电归一化、阈值与形态判断、亚像素位置。

LSM6DSR_Config.c 与 LSM6DSR_Config.h：六轴 IMU 读写与物理量转换。

ABEncoder.c 与 ABEncoder.h：编码器测速，输出 speed_left、speed_right。

ADC_get.c 与 ADC_get.h：多路复用器选通十六路光电到 ADC 通道 1（PA1）；另在 MuxADC_SampleAll 末尾采样 PA0（ADC 通道 0），写入 g_mux_adc_values 下标 16，供母线或分压电压换算。主程序中 BDI_V 等于该路原始值乘以 0.00426508726，即原先已具备电压采集链路。

Key_Scan.c 与 Key_Scan.h：按键扫描与事件。

M3PWM.c 与 M3PWM.h：TIM2 一路 PWM，占空比参数为零到一千，用于整车主功率或限速设定（例如按键 K1 中 M3PWM_SetDutyCycle(950)）。与电机桥 TIM1 的零到一万占空比是两套不同定时器。

OLED.c 与 OLED.h：OLED 底层显示。

RGB_Led.c 与 RGB_Led.h：RGB 指示灯。

Uart_Config.c 与 Uart_Config.h：串口与通讯超时。

TelemetryScreen.c 与 TelemetryScreen.h：上层遥测界面（四行十六列，标签为中文点阵加 ASCII 数值），见 Software 目录 log。OLED_CN.c 与 OLED_CN.h：八字模与绘制接口，配合 OLED_DrawBitmap16x16 使用。

pose.c 与 pose.h：姿态辅助，预留互补滤波与四元数等接口。


三、电压与“电量”说明（与原先设计的关系）

原先工程已有电压采样：ADC_get 在每次 SysTick 周期中调用 MuxADC_SampleAll，除十六路光电外，对 PA0 做一次转换，结果放在 g_mux_adc_values[16]。main 中将其换算为 BDI_V。因此可以显示实时电压。若将电压映射为“剩余百分比”，只能得到粗略估计，受负载压降、电池化学曲线等影响；精确 SOC 需要库仑计或专用电量计芯片，单靠电压无法保证准确。


四、整体控制逻辑（运行流程）

上电后依次初始化各模块，SysTick 作为心跳周期。

主循环中处理按键、更新 BDI_V、刷新 TelemetryScreen 等。

SysTick 中完成 IMU、ADC、寻线、PID 与保护逻辑。

异常与安全：长时间丢线或串口超时等会关电机、清零 star_car。


五、快速上手与移植建议

用 Keil 打开 Project.uvprojx 编译下载。若改硬件，请同步修改电机与传感器引脚、ADC 标定、IMU 接口等。调参可修改 PID 与 BlackPoint_Finder 内参数。
