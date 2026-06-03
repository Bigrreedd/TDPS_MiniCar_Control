# TDPS Background 资料目录

本目录保存 TDPS 微缩智能车项目的背景资料、课程说明、项目背景、硬件选型、固件迁移和 PCB 资料。

> 注意：`TDPS_Background/` 为本地背景资料目录，按项目规则被 Git 忽略。

## 目录结构

```text
TDPS_Background/
├── README.md
├── 00_course/                       课程任务书
│   └── L1b_Design-Tasks_An-Overview_2025-2026 Final.pdf
├── 01_overview/                     项目整体背景与调试说明
│   ├── TDPS_完整背景资料.md
│   └── TDPS_下板PID调参与双层恢复说明.md
├── hardware/                        硬件选型与器件资料
│   ├── hardware_selection.md
│   ├── motor_pwm_duty_limit_analysis.md
│   ├── MPU6050/                     IMU/陀螺资料（压缩包 + 已拍平的源码）
│   │   ├── GY-32-MMA7361模块发送资料.rar
│   │   ├── GY521mpu-6050资料.zip
│   │   └── MPU和OLED材料/MPU6050_OLED/   MPU6050+OLED 示例源码
│   ├── motor_driver_board/          新电机驱动板照片
│   ├── motor_specs/                 电机参数图（车机、风扇电机）
│   ├── battery_specs/               电池参数图
│   └── encoder_schematic/           编码器原理图与网表
├── 03_firmware/                     固件迁移记录
│   ├── README.md
│   ├── competition_branch/competition_branch_plan.md
│   ├── current_new_hardware/current_firmware_file_structure.md
│   └── legacy_reference/legacy_module_reference.md
├── 04_pcb/                          PCB 设计资料
│   ├── C17488971_..._DRV8701EVM_..._WJ679115(1).PDF
│   ├── datasheets/ucc27517.pdf
│   ├── fan_driver/                  风扇电机驱动分析
│   │   ├── fan_mosfet_driver_analysis.md
│   │   ├── fan_driver_teammate_explanation.md
│   │   └── r6_gate_pulldown_drive_check.md
│   ├── main_board/                  主板原理图与 3D 渲染
│   │   ├── 最新的原理图.pdf
│   │   └── 3D渲染图.png
│   ├── gen3_board/                  三代板原理图/BOM/网表
│   │   ├── SCH_Schematic5_2026-06-01(1).pdf
│   │   ├── BOM_三代板_Schematic5_2026-06-01(3).xlsx
│   │   ├── Netlist_Schematic5_2026-06-01(2).tel
│   │   └── tel_to_kicad_netlist.py
│   ├── grayscale_sensor/感为八路灰度传感器手册.pdf
│   └── notes/new_main_board_pin_migration.md
└── 05_burned_hardware/              烧毁后的硬件记录与说明
    ├── 看二合一.epro2
    ├── 下层C8T6主控引脚分配图.docx
    └── Netlist_Schematic7_2026-06-02.net
```

## 分类说明

- `00_course/`：课程任务书、课程背景。
- `01_overview/`：项目整体背景、系统说明、PID 调参与双层恢复说明。
- `hardware/`：电机、电池、编码器、IMU 等硬件选型和器件资料。
- `03_firmware/`：固件迁移记录、分支职责、当前与旧代码结构说明。
- `04_pcb/`：PCB 原理图、BOM、网表、器件数据手册和驱动分析。
- `05_burned_hardware/`：烧毁后硬件的原理图、引脚分配和说明。

## 重点资料快查

- PID 调参与双层恢复：`01_overview/TDPS_下板PID调参与双层恢复说明.md`
- 电机 PWM 占空比上限：`hardware/motor_pwm_duty_limit_analysis.md`
- 硬件选型记录：`hardware/hardware_selection.md`
- 竞赛分支说明：`03_firmware/competition_branch/competition_branch_plan.md`
- 当前固件结构：`03_firmware/current_new_hardware/current_firmware_file_structure.md`
- 风扇驱动分析：`04_pcb/fan_driver/fan_mosfet_driver_analysis.md`
- 新主板引脚迁移：`04_pcb/notes/new_main_board_pin_migration.md`
- 三代板原理图：`04_pcb/gen3_board/SCH_Schematic5_2026-06-01(1).pdf`

## 命名约定

- 顶层用编号前缀（00/01/03/04/05）控制阅读顺序；`hardware/` 因目录被占用暂保留原名，解锁后可改为 `02_hardware`。
- 子目录统一英文短名；原始厂商资料（压缩包、手册、原理图）保留原文件名。
- MPU6050 资料保留压缩包，已删除可重新解压的重复解压目录，仅保留一份示例源码。
