# TDPS Background 资料目录

本目录保存 TDPS 微缩智能车项目的背景资料、课程说明、调参日志、硬件选型、固件迁移和 PCB 资料。

> 注意：`TDPS_Background/` 整体被 Git 忽略；**例外**：已被强制纳管的关键文件（调参存档 `01_overview/archives/`、各分析文档等）会随提交保存。

## 目录结构（2026-06-05 整理版）

```text
TDPS_Background/
├── README.md                        本索引
├── 00_course/                       课程任务书与赛道地图
│   ├── L1b_Design-Tasks_An-Overview_2025-2026 Final.pdf
│   ├── 赛道地图_全局预览图.png
│   └── 赛道地图_右半边图.png
├── 01_overview/                     项目背景 + 调参作战文档（活跃区）
│   ├── PID_TUNING_LOG.md            ★ 调参主日志（每轮测试全量记录，只追加）
│   ├── archives/                    ★ 关键轮次存档（绑定代码快照，git 纳管）
│   ├── PHASE2_ROADMAP.md            阶段路线图（注意：路线顺序叙述有误，实测 U 弯最前）
│   ├── PARAM_EFFECT_MAP.md          参数-效果图谱
│   ├── DEBUG_STATE.md / PROJECT_STATUS.md / ROAD_TEST_DAY1.md
│   ├── code_review_report.md
│   ├── TDPS_完整背景资料.md
│   └── TDPS_下板PID调参与双层恢复说明.md
├── 02_hardware/                     硬件选型与器件资料（原 hardware/，编号归位）
│   ├── hardware_selection.md        硬件选型记录
│   ├── motor_pwm_duty_limit_analysis.md  ★ 三电机占空比上限分析（06-05 第 7 章定版）
│   ├── battery_specs/               电池参数图（Tattu 3S 850mAh 75C）
│   ├── motor_specs/                 电机参数图（车机 50TPA / 风扇电机 30TPA）
│   ├── motor_driver_board/          电机驱动板照片
│   ├── encoder_schematic/           编码器原理图与网表
│   └── MPU6050/                     IMU 厂商资料（压缩包 + 示例源码）
├── 03_firmware/                     固件结构与跨板协作
│   ├── README.md
│   ├── competition_branch/          竞赛分支计划
│   ├── current_new_hardware/        当前固件文件结构
│   ├── legacy_reference/            旧模块参考
│   └── 与上层板通信_雷达loraesp/      ★ 队友通信约定（说明文件.md + 沟通截图 + car_designer_prompt.md）
├── 04_pcb/                          PCB 设计资料
│   ├── 2合1/                        ★ 当前两板方案（Schematic7 06-05）
│   │   ├── SCH_Schematic7_2026-06-05.pdf
│   │   ├── 双层板上面负责灰度和IMU的板子/    主板 BOM/网表/PCB
│   │   └── 双层板下面负责电机驱动的板子/    PCB6 BOM/网表/SCH（DRV8701×2）
│   ├── datasheets/                  器件手册（DRV8701EVM 规格书、UCC27517）
│   ├── fan_driver/                  风扇驱动分析（MOS/栅极/续流）
│   ├── gen3_board/                  三代板（含 _另版下载 SCH；重复 BOM/网表已查重删除）
│   ├── main_board/                  主板原理图与 3D 渲染
│   ├── grayscale_sensor/            感为八路灰度传感器手册
│   └── notes/                       引脚迁移笔记
└── 05_burned_hardware/              烧毁硬件事故记录（Schematic7 网表/引脚分配/工程文件）
```

## 重点资料快查

- **调参主日志**：`01_overview/PID_TUNING_LOG.md`（含 06-05 全天 14 轮 + 审查裁决 + 功率审查）
- **关键轮次存档**：`01_overview/archives/`（里程碑/U弯零丢线/S-mode 依据，git tag `milestone-20260605-reach-s-curve`）
- **三电机占空比上限**：`02_hardware/motor_pwm_duty_limit_analysis.md` 第 7 章（轮 2000 禁上调；风扇 0/20/50 双层钳位，代码 LHX/lower-pid b0c8d62）
- **队友通信约定**：`03_firmware/与上层板通信_雷达loraesp/说明文件.md`（拱门 ESP→C8T6 通知等）
- **当前两板 PCB**：`04_pcb/2合1/`（上板灰度+IMU / 下板 PCB6 电机驱动）
- **风扇驱动分析**：`04_pcb/fan_driver/fan_mosfet_driver_analysis.md`
- **赛道地图**：`00_course/赛道地图_全局预览图.png`（真实路线顺序以实测为准：Start→Y1→U弯→Y2→S弯→三方框→四圆→雷达箱）

## 命名约定

- 顶层编号前缀（00~05）控制阅读顺序；`02_hardware` 已于 06-05 由 `hardware/` 归位。
- 子目录英文短名优先；厂商原始资料（压缩包、手册、原理图）保留原文件名；中文描述性目录名用于板卡职责（如 `2合1/双层板下面...`）。
- 查重原则：字节级相同（md5）才删除，不同版本一律保留并加 `_另版` 后缀。

## 整理变更记录（2026-06-05）

| 旧路径 | 新路径 | 备注 |
|---|---|---|
| `hardware/` | `02_hardware/` | 编号归位 |
| `hardware/与上层板通信-雷达lora相关文件/car_designer_prompt.md` | `03_firmware/与上层板通信_雷达loraesp/` | 合并重复主题目录 |
| `04_pcb/C17488971_..._DRV8701EVM_规格书...PDF` | `04_pcb/datasheets/DRV8701EVM_规格书_C17488971.PDF` | 归入手册目录 |
| `04_pcb/最终可能会用的一块板子/` | 删除（BOM/网表与 gen3_board md5 相同）；SCH 不同版 → `gen3_board/SCH_..._另版下载.pdf` | 查重 |
| `00_course/全局预览图.png` 等 | `00_course/赛道地图_*.png` | 可检索性 |
| `hardware/battery_specs/56355e56....png` | （队友已删，本次确认移出 git） | 850mAh 75C.png 仍在 |
