# TDPS 项目当前状态

> 更新时间：2026-06-03 23:35
> 分支：LHX/upper-test (已提交并 push)

---

## 一、今日完成工作（2026-06-03）

### 1. 速度 PID 调优与稳定性验证

**问题**：40 cnt/s 出现 bang-bang 振荡，out 在 0 ↔ 200 间剧烈跳变。

**解决方案**：
- **Ki 三次方缩放**：从 `Ki = 14.0 × s²` 改为 `Ki = 14.0 × s³`，在 40 cnt/s 时 Ki = 0.896，50 cnt/s 时 Ki = 1.75
- **死区分离**：起步死区 900，保持死区 820，速度 > 10 cnt/s 切换
- **最小正向输出**：当目标 > 0 且 PID 输出 < 5.0 时，强制输出 5.0，防止轮子停转

**验证结果**：
- 40 cnt/s：Kp=0.8 Ki=0.896 Kd=0.8，稳定无振荡
- 50 cnt/s：Kp=1.0 Ki=1.75 Kd=1.0，最优平衡点（推荐）
- 60 cnt/s：Kp=1.2 Ki=3.024 Kd=1.2，稳定

**当前台架目标**：`BENCH_FIXED_TARGET_CPS = 50.0f`

### 2. 位置环方向验证

**测试场景**：台架模式，`BENCH_POSITION_TEST_ENABLE = 1`，限幅 ±30

**验证结果**（50 cnt/s）：
- 左偏（ch2 压黑）→ 右轮快（+差速）
- 右偏（ch3 压黑）→ 左轮快（−差速）
- 方向全对，差速明显

### 3. ESP32 通信模块

**完成内容**：
- 协议头文件：`User/ESP32_Comm.h`
- 状态机解析器：`User/ESP32_Comm.c`
- 帧格式：A5 5A | ver | type | seq(LE) | len(LE) | payload | checksum
- API：`ESP32_SendAtPosition()`、`ESP32_GetDecision()`、`ESP32_GetRadarData()`
- 弱符号接口：`ESP32_UART_Init()`、`ESP32_UART_SendByte()` 等待用户指定 UART

**待用户确认**：使用哪个 UART（USART1/USART2/USART3）

### 4. Day 1 测试计划

**文档**：`ROAD_TEST_DAY1.md`

**测试内容**：
- Phase 1：编码器标定（轮周 × N 圈 → 真实脉冲/cm）
- Phase 2：直线测试（50 cnt/s 和 60 cnt/s）
- Phase 3：手动 S 弯（验证差速方向）
- Phase 4：启停测试（死区分离验证）

**硬件清单**：上下板、电池、USB-TTL、卷尺、Keil 下载器

**固件配置**：`BENCH_FIXED_SPEED_ENABLE=1`、`SENSOR_COUNT=6`、`USART3_DEBUG_ON_PB10=1`

---

## 二、分支结构与远程同步状态

### 已 push 的分支

| 分支 | 描述 | 远程状态 |
|------|------|---------|
| `LHX/upper-test` | 上板速度 PID 调优 + ESP32 模块 | ✅ 已 push |
| `LHX/upper` | 上板功能分支（较旧） | ✅ 已 push |
| `LHX/lower` | 下板纯执行器模式（旧） | ✅ 已 push |
| `LHX/lower-pid` | 下板速度环调参 + 执行器模式 | ✅ 已 push |
| `LHX/pcb2` | PCB 相关 | ✅ 已 push |
| `LHX/competition` | 比赛集成分支 | ✅ 已 push |
| `LHX/legacy-motor` | 旧电机实现参考 | ✅ 已 push |
| `LHX/sensor` | 传感器相关 | ✅ 已 push |
| `version-0` | 早期基线 | ✅ 已 push |

**所有本地分支已全部同步到远程仓库。**

### Background 目录状态

`TDPS_Background/` 目录已加入 git 跟踪并 push，包含：
- **课程资料**：`00_course/L1b_Design-Tasks_An-Overview_2025-2026 Final.pdf`
- **项目背景**：`01_overview/` 完整背景文档、下板 PID 调参说明
- **固件迁移**：`03_firmware/` 分支说明、文件结构
- **PCB 资料**：`04_pcb/` 原理图、数据手册、驱动分析
- **硬件资料**：`hardware/` 硬件选型、电机 PWM 分析、ESP32 通信协议、MPU6050 资料

**保留理由**：所有文档均包含项目核心参考信息，无冗余文件。

---

## 三、关键技术参数汇总

### 速度 PID 参数（增量式，自动缩放）

| 目标速度 | scale | Kp | Ki | Kd | 场景 |
|---------|-------|----|----|----|----|
| 40 cnt/s | 0.5 | 1.0 | 1.75 | 1.0 | 低速极限 |
| 50 cnt/s | 0.625 | 1.25 | 3.42 | 1.25 | **推荐基准** |
| 60 cnt/s | 0.75 | 1.5 | 5.91 | 1.5 | 中速直线 |
| 80 cnt/s | 1.0 | 2.0 | 14.0 | 2.0 | 高速直线 |

**基准增益**（s=80 cnt/s）：Kp=2.0, Ki=14.0, Kd=2.0

**运行时缩放**：
```c
float s = i_speed / 80.0f;
s = clamp(s, 0.3f, 1.5f);
Kp = 2.0f × s;
Ki = 14.0f × s × s × s;  // 三次方
Kd = 2.0f × s;
```

### 死区补偿

| 参数 | 值 | 说明 |
|------|---:|------|
| `MOTOR_START_DEADZONE_L` | 900.0 | 左轮起步死区 |
| `MOTOR_START_DEADZONE_R` | 900.0 | 右轮起步死区 |
| `MOTOR_HOLD_DEADZONE_L` | 820.0 | 左轮保持死区 |
| `MOTOR_HOLD_DEADZONE_R` | 820.0 | 右轮保持死区 |
| `MOTOR_HOLD_SPEED_CPS` | 10 | 速度阈值（cnt/s）|

**逻辑**：速度 > 10 cnt/s 时用 hold 死区，否则用 start 死区。

### 位置环参数（台架测试模式）

| 参数 | 值 | 说明 |
|------|---:|------|
| `BENCH_POSITION_TEST_ENABLE` | 1 | 台架位置环测试开关 |
| `BENCH_POSITION_TEST_CORRECTION_LIMIT` | 30.0 | 台架差速限幅 |
| `POSITION_LOOP_ENABLE` | 0 (上路前改 1) | 位置环总开关 |

**当前台架逻辑**：限幅 ±30，仅在 `BENCH_FIXED_SPEED_ENABLE=1` 时生效。

### 传感器配置

| 参数 | 值 | 说明 |
|------|---:|------|
| `SENSOR_COUNT` | 6 | 可用灰度传感器数量 |
| `USART3_DEBUG_ON_PB10` | 1 | USART3 占用 ch6，ch6 停用 |
| 回中位置 | ch2 与 ch3 之间 | 位置 2.5（0-5 范围）|

---

## 四、明日工作计划（Day 1：2026-06-04）

### 上午：台架基础验证

**目标**：确认编码器标定、速度闭环稳定性、差速方向正确性。

**流程**：
1. 编码器标定（记录轮周 → 推 N 圈 → 记 OLED 累计脉冲 → 算 `ENCODER_TICKS_PER_CM`）
2. 直线 50 cnt/s（架空，观察速度稳定性）
3. 直线 60 cnt/s（架空，观察速度平滑性）
4. 手动 S 弯（地面慢推，验证差速方向）
5. 启停测试（验证死区分离）

**预期结果**：
- 编码器标定值记录（更新到 `User/ABEncoder.c`）
- 50/60 cnt/s 速度曲线平滑无振荡
- 左偏 → 右快，右偏 → 左快（方向正确）
- 启停平滑无抽动

### 晚上：分支合并与参数同步

**目标**：将 `LHX/upper-test` 的调优参数合并到 `LHX/competition`。

**任务**：
1. 合并速度 PID 参数（三次方 Ki、死区分离、最小输出）
2. 合并 ESP32 通信模块（确定 UART 后集成）
3. 修复 `Path.c` 距离阈值（根据编码器标定结果）
4. 修复速度单位兼容性（competition 分支用 2ms 增量）
5. 测试合并后的 competition 分支（低速闭环，固定目标）

**预期结果**：`LHX/competition` 可用于 Day 2 全图测试。

---

## 五、待办事项（按优先级）

### P0 - Day 1 必须完成

- [x] 速度 PID 调优（40/50/60 cnt/s）
- [x] 位置环方向验证
- [x] ESP32 通信模块（协议解析器）
- [x] Day 1 测试计划文档
- [x] 所有分支 push 到 GitHub
- [ ] 编码器标定（Day 1 上午）
- [ ] 台架基础验证（Day 1 上午）
- [ ] 分支合并与参数同步（Day 1 晚上）

### P1 - Day 2-4 测试期间

- [ ] 用户指定 ESP32 使用的 UART
- [ ] 实现 `ESP32_UART_Init()` 和 `ESP32_UART_SendByte()`
- [ ] 在 `main.c` 中集成 ESP32 调用（特定位置发送，接收决策）
- [ ] 全图测试（competition 分支）
- [ ] 修复竞赛流程中发现的问题

### P2 - 竞赛功能补全

- [ ] 圆形干扰区避让策略（四圆区域最短路径）
- [ ] 雷达决策逻辑（ESP32 → STM32 决策传递）
- [ ] LoRa 门传输（250cm 和 700cm 位置）
- [ ] 终点识别与停车

### P3 - 优化与调试

- [ ] 左右轮速度差补偿（WHEEL_BALANCE_KP=8.0）
- [ ] 高速弯道速度自适应
- [ ] IMU 陀螺仪融合（位置环 gyro_kd）
- [ ] 电池电压监控与低压保护

---

## 六、已知约束与安全红线

### 硬件约束

| 项目 | 限制 | 说明 |
|------|------|------|
| 可用灰度传感器 | 6 路 | ch6 被 USART3 占用 |
| 电机占空比上限 | 2000/10000 | `MOTOR_DUTY_SAFE_MAX` |
| 风扇状态 | 禁用 | `FanMotor_SafetyLock`，不输出 PWM |
| 速度下限 | 40 cnt/s | 低于此值接近电机最低持续速度 |

### 调参红线

- **不得通过提高目标速度换取测速精度**（电机烧毁风险）
- **不得将 `MOTOR_DUTY_SAFE_MAX` 提高到 2000 以上**（电流/温升未验证）
- **不得启用风扇 PWM**（功率级未验证）
- **上路前必须将 `BENCH_FIXED_SPEED_ENABLE` 改回 0**（否则无速度规划）

---

## 七、关键文件清单

### 核心控制代码

| 文件 | 功能 |
|------|------|
| `User/PID_Controller.c` | 速度环、位置环、台架模式 |
| `User/main.c` | 主循环、电机控制、遥测输出 |
| `User/Path.c` | 赛道状态机、速度规划 |
| `User/ESP32_Comm.h/c` | ESP32 通信协议解析器 |
| `User/BlackPoint_Finder.h/c` | 黑线识别（6 路灰度）|
| `User/ABEncoder.c` | 编码器测速与里程 |

### 配置头文件

| 文件 | 配置项 |
|------|--------|
| `User/stm32f10x_conf.h` | `USART3_DEBUG_ON_PB10` |
| `User/PID_Controller.c` | `BENCH_FIXED_SPEED_ENABLE`、`BENCH_FIXED_TARGET_CPS` |

### 测试文档

| 文件 | 用途 |
|------|------|
| `ROAD_TEST_DAY1.md` | Day 1 台架测试计划 |
| `DEBUG_STATE.md` | 历史调参记录（参考）|
| `PROJECT_STATUS.md` | 本文档（项目状态）|

---

## 八、Git 提交历史（最近 5 条）

```
c5b834d (HEAD -> LHX/upper-test, origin/LHX/upper-test) docs: add debug state history + keil project update
4e09877 feat: stable 40/50/60 bench test + ESP32 comm module
0bdc88c fix(upper): lock fixed target speed and disable wheel balance for bench test
259176a fix(upper): make closed loop bench test forward only
cea469c test(upper): switch back to closed loop mode
```

---

## 九、仓库统计

- **总提交数**：约 150+ commits
- **活跃分支**：9 个
- **代码规模**（上板 upper-test）：
  - text: ~18KB
  - data: ~136B
  - bss: ~2KB
  - Flash 占用：约 18KB / 64KB
  - RAM 占用：约 2KB / 20KB

---

**所有工作已就绪，明天加油！🚗💨**
