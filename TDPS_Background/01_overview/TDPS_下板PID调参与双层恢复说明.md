# TDPS 小车 下板 PID 调参 + 上下双层恢复说明

> 仓库：`D:\STUDY\TDPS\TDPS_MiniCar_Control`
> 主要分支：下板 `LHX/lower-pid`，上板 `LHX/upper-test`
> 文档更新时间：2026-06-03

本说明把本次会话从「下板 PID 调参」到「准备恢复上下双层控制」的全过程压缩成一份可复查文档，含分支结构、git 提交历史、关键代码位置和测试流程。

## 0. 当前进度速览（最重要）
**所处阶段**：下板低速 PID 已调稳并固化默认参数；上板开环链路已验证有阶梯速度变化，当前已切回闭环模式，准备架空轮子做闭环速度验证。

**已完成**：
- 下板速度环单向输出、200ms 测速窗口、串口空闲自动提交，全部修复并提交。
- 下板稳定默认参数 target=80 / Kp=6 / Ki=0.5 / Kd=0 已写入代码。
- 下板 `LHX/lower-pid` 默认已切为纯执行器模式：`LOWER_PID_TUNE=0`（提交 `43cfea4`）。
- 仓库清理（.gitignore + 移除 Project.uvoptx）已 cherry-pick 同步到全部 9 条分支。
- 上板 `LHX/upper-test` 已切到开环测试模式，开环占空比提高到 1200/1500/1800。
- 上板开环按键已改为按下即生效，并在 OLED 显示 raw 按键位与当前 duty（提交 `defa660`）。
- 上板速度 PID 起点已改为保守值 `Kp=6 / Ki=0.5 / Kd=0`（提交 `7aaf485`）。
- 上板开环 OLED 跳变已修复：开环模式使用独立稳定页面，不再和普通遥测页轮流清屏（提交 `050c9b1`）。
- 上板开环 duty 被错误钳到 PID 调节上限的问题已修复：K1/K2/K3 现在会真实下发 1200/1500/1800（提交 `80322ba`）。
- 上板已从开环测试切回闭环模式：`OPENLOOP_TEST_ENABLE=0`（提交 `cea469c`）。
- 闭环架空测试出现疯狂抽动后，已改成速度闭环单向架空测试：默认禁用位置环 `POSITION_LOOP_ENABLE=0`，闭环发送前禁止反向 `CLOSED_LOOP_REVERSE_ENABLE=0`（提交 `259176a`）。
- 架空闭环再测仍单向加速，根因是 Path 状态机靠编码器假里程把目标速度不断推高（SEARCH 90→STRAIGHT 140→LINE_FOLLOW 115…），外加 wheel_balance 放大左右速度差。已加 `BENCH_FIXED_SPEED_ENABLE` 架空模式：目标固定 80 cnt/s、wheel_balance 强制关（提交 `0bdc88c`）。

**正在卡的点**：开环链路已通过；闭环架空初测单向加速，根因已定位为 Path 假里程推动态目标。已加架空固定速度开关，等待重新烧上板验证。

**下一步要用户做的**：
1. 下板烧最新 `LHX/lower-pid`（默认 `LOWER_PID_TUNE=0`）。
2. 上板烧最新 `LHX/upper-test`（闭环模式，`OPENLOOP_TEST_ENABLE=0`）。
3. 架空轮子，按 K1 启动闭环速度测试，回报 OLED 轮速、是否仍抽动/拉满/停车。

**未决/待办**：
- 左右轮速度差约 10~15 cnt/s，双层阶段需要补偿。
- `LHX/lower` 是否删除，等双层验证通过再定。
- 4 条分支本地 `ahead 1`，尚未 push。
- 主工作区当前在 `LHX/upper-test`，且 `Project.uvprojx` 有本地 modified 状态但 `git diff` 内容为空；不要误提交这个 Keil/行尾状态。

## 0.5 当前接手点（2026-06-03）
- 上板最新提交：`0bdc88c fix(upper): lock fixed target speed and disable wheel balance for bench test`。
- 下板最新提交：`43cfea4 fix(lower): default to executor mode for dual board test`。
- 当前上板模式：`OPENLOOP_TEST_ENABLE=0`，闭环；架空速度闭环验证用 `BENCH_FIXED_SPEED_ENABLE=1`（固定 80 cnt/s），`POSITION_LOOP_ENABLE=0`、`CLOSED_LOOP_REVERSE_ENABLE=0`。
- 已知结果：开环 K1/K2/K3 已确认有阶梯速度变化，板间链路、电机输出、编码器回传基本打通。
- 刚发生的问题：闭环架空再测单向加速，根因是 Path 状态机靠编码器假里程把目标一路推高（`SPEED_SEARCH=90` → `SPEED_STRAIGHT=140` → `SPEED_LINE_FOLLOW=115` …），外加 `wheel_balance` 把左右速度差放大成 ±300 差分输出。已加 `BENCH_FIXED_SPEED_ENABLE` 固定目标 ± 禁用 wheel_balance 修复（`0bdc88c`）。
- 下次动作：架空轮子按 K1，验证速度闭环是否平稳保持在 ~80 cnt/s，不再持续加速。

## 1. 本次目标
- 修稳下板低速速度环 PID 调参固件。
- 厘清编码器单位，增加 RAW/窗口遥测。
- 防止低速调参时电机反转抽动。
- 串口命令无换行也能可靠提交。
- 调稳后恢复上下双层控制，先做开环链路验证。

## 1.5 分支结构讲解
二合一板（上板=大脑，下板=执行器）共用一个仓库，按职责分多条分支：

| 分支 | 角色 | 说明 |
| --- | --- | --- |
| `LHX/lower-pid` | 下板 · 当前主用 | 下板速度环脱机调参 harness + 纯执行器模式，靠 `LOWER_PID_TUNE` 开关切换。本次调参都在这里。 |
| `LHX/lower` | 下板 · 旧基线 | 纯执行器旧版本，没有本轮诊断/保护开关。建议双层验证通过后再删。 |
| `LHX/upper-test` | 上板 · 当前主用 | 上板大脑测试分支，含开环测试开关 `OPENLOOP_TEST_ENABLE`、编码器单位修正、死区前馈。 |
| `LHX/upper` | 上板 · 较稳版本 | 上板功能分支。 |
| `LHX/legacy-motor` | 参考 | 旧电机/编码器实现，用来对比确认下板编码器逻辑一致。 |
| `LHX/competition` | 集成 | 比赛集成分支。 |
| `LHX/sensor` | 传感器 | 灰度/传感相关。 |
| `LHX/pcb2` | 硬件 | PCB 第二版相关。 |
| `version-0` | 基线 | 早期基线快照。 |

开关与单板的关系：
- 下板 `LOWER_PID_TUNE=1`：跑速度环调参（按键+串口调 PID，关看门狗，需架空轮子）。
- 下板 `LOWER_PID_TUNE=0`：纯执行器（收 MOTOR_CMD、回传 ENC_FEEDBACK、启用看门狗）。
- 上板 `OPENLOOP_TEST_ENABLE=1`：按键直接给固定占空比，不跑 PID，验证占空比→编码器是否单调。
- 上板 `OPENLOOP_TEST_ENABLE=0`：正常循迹（位置环+速度环+Path）。

## 2. 编码器单位（关键概念）
- SysTick 500Hz，`speed_left/speed_right` 是 **2ms 原始脉冲增量**（遥测里的 RAW）。
- `left_encoder_cnt/right_encoder_cnt` 是累计计数，用于按窗口换算 **cnt/s**（遥测里的 CPS）。

## 2.5 编码器精度问题（重点，专门记录）
这是本次会话花最多时间讨论的问题，结论分三层：

**现象**：
- 用户觉得轮子转得不慢，但编码器原始读数变化很慢，精确不到个位数。
- 遥测里 CPS 都是以 10 为基本单位跳（70 / 80 / 90），调参手感粗。

**原因分析**：
1. RAW 是 2ms 增量，低速下本来就只有 0 或 1，属正常物理现象（80 cnt/s × 2ms = 0.16 tick）。
2. CPS 由累计计数按窗口换算。窗口越短，1 个 tick 折算的 cnt/s 越大，分辨率越粗：
   - 100ms 窗口：1 tick = 10 cnt/s（所以以 10 为单位跳）。
   - 200ms 窗口：1 tick = 5 cnt/s。
3. 编码器硬件本身没坏：与 `LHX/legacy-motor` 初始化完全一致，dt 稳定，左右 tick 对称合理。

**已采取的安全改进**：
- 把测速/打印窗口从 100ms 加长到 200ms（提交 `04c0921`），分辨率由 10 cnt/s 细化到 5 cnt/s。
- 实测 200ms 下 tick 约 9~15，CPS 能看到 70/75/80/85 这种更细的台阶。
- 代价：控制反馈每 200ms 更新一次，响应更慢但更平滑。

**安全红线（用户明确要求，务必遵守）**：
- **绝对不能靠提高目标速度来换取测速精度，电机有烧毁风险。**
- 低速调参只能通过测速窗口、滤波、显示口径改善分辨率，不能提速。

**关于“一圈 100 变 1000”的真相**：
- 真实分辨率只能来自硬件脉冲数，软件无法把 100 个真实计数变成 1000 个真实计数。
- 当前 TIM 已用 `TIM_EncoderMode_TI12`，已是比较完整的 AB 相解码方式。
- 想真正提升真实分辨率，只有三条硬件路：
  1. 换更高 PPR/CPR 的编码器（最干净）。
  2. 把编码器装在电机轴侧而非轮轴侧，借减速比放大 tick。
  3. 排查是否漏边沿（做“手转一圈累计计数”测试：若理论应更高但实测低很多，才是接线/模式问题）。
- 软件插值（滑动平均、IIR、周期测量）只能让显示更平滑，不增加真实位置信息。
- 用户当前决定：先不动硬件，按现状（200ms 窗口）推进调参。

## 3. 下板调参 harness（LOWER_PID_TUNE=1）
物理按键：
- K1 运行/停止
- K2 停止
- K3 目标 +10
- K4 目标 -10（带下限）

串口命令（USART2, 115200）：
- `p<Kp>` `i<Ki>` `d<Kd>` 设增益
- `t<目标 cnt/s>` 设目标速度（带下限）
- `g` 运行，`s` 停止，`?` 查看
- 支持无换行：空闲 20ms 自动提交。

遥测格式：
```
T=目标 RAWL/RAWR=2ms增量 CPSL/CPSR=cnt/s out=速度环输出 tickL/tickR=窗口内计数 dt=窗口ms
```

## 4. 已确认的稳定默认参数（写入代码）
```c
TUNE_DEFAULT_TARGET = 80   // cnt/s
Kp = 6.0
Ki = 0.5
Kd = 0.0
TUNE_SPEED_WIN_MS = 200    // 1 tick = 5 cnt/s
TUNE_MIN_TARGET = 70
```
表现：单向输出无反转抽动，CPS 约 50~80，右轮比左轮快约 10~15 cnt/s，out 由积分平滑爬升。

## 4.5 调试时间线（事无巨细）
按本次会话实际发生顺序记录：

1. **起点**：延续之前的下板编码器/PID 调试，分支 `LHX/lower-pid`。
2. **编码器“变慢/精度低”疑问**：解释 RAW 是 2ms 增量，低速下 0/1 属正常；对比 `LHX/legacy-motor` 确认编码器初始化（TIM3 PA6/PA7、TIM4 PB6/PB7、TI12 编码器模式）一致，硬件没坏。
3. **PID 剧烈抖动**：用户遥测出现 `CPS=90 → out=-130`、`CPS=-20 → out=1000` 来回跳。根因是 PID 负输出被 `tune_apply_motor` 当成反向，前后抽动。修复=速度环单向前进、负输出钳 0（提交 `a1fbd34`）。
4. **单向修复后**：遥测变稳，out 仅正、CPS 40~70 平滑。
5. **运行时调 `p6 i0.3 d0`**：稳定但速度偏低，out 由 ~115 缓慢爬到 ~244。
6. **`t80` 不生效**：发了 `t80` 但无 `OK target=80`，遥测仍 `T=70`。判断终端没发换行。修复=串口空闲 20ms 自动提交（提交 `43424ea`）。
7. **`t80` 生效**：回 `OK target=80 cnt/s`，遥测 `T=80`，两轮 70~80，无抽动。
8. **固化默认**：把 target=80、Kp=6、Ki=0.3 写入代码（提交 `7f6d789`）。
9. **精度讨论**：用户问能否让原始读数更细。说明 100ms 下 1 tick=10 cnt/s。
10. **用 simplify skill 审代码**：拆出 `tune_clamp_forward / tune_add_forward_deadzone / tune_wheel_duty`，修正 `Key_Scan.h` 中 K1/K2 引脚注释（提交 `618591e`），功能不变。
11. **明确安全约束**：用户强调**绝不能靠提高目标速度解决精度，电机会烧**。改为加长测速窗口到 200ms，1 tick=5 cnt/s（提交 `04c0921`）。已存入记忆。
12. **窗口加长后实测**：`p6 i0.3` 稳但速度低，左轮比右轮慢；试 `i0.5` 更合适，速度上来且不抖。把默认 Ki 提到 0.5（提交 `b59a96f`）。
13. **源头精度讨论**：用户希望“一圈 100 变 1000”。说明真实分辨率只能靠硬件（更高 PPR / 装电机轴侧 / 排查漏边沿），软件插值不增真实信息。用户决定先不管，按现状推进。
14. **转入双层恢复**：讨论烧原 `LHX/lower` 还是用 `lower-pid` 置 0。结论用 `lower-pid` 置 `LOWER_PID_TUNE=0`。
15. **分支取舍**：建议保留 `lower-pid`，`lower` 暂留待验证后删。
16. **仓库清理**：发现 `Project.uvoptx`、`image_reader/`、`video_reader/` 是无关本地产物。新增 .gitignore 规则、从索引移除 uvoptx（提交 `3cdc972`），并 cherry-pick 同步到全部分支。
17. **上板按钮说明**：讲解正常模式与开环模式按键差异。
18. **开环模式启用**：把 `OPENLOOP_TEST_ENABLE` 置 1（提交 `ad2399f`）。
19. **K1/K2/K3 无反应**：排查发现开环占空比 500/800/1000 低于死区，提高到 1200/1500/1800（提交 `9a1e29a`），等待用户复测。

## 5. Git 提交历史

### 5.1 下板 `LHX/lower-pid`（从早到晚）
```
a4f73b5 feat(lower): add standalone speed PID tuning mode   # 新建脱机调参模式
9d6c34d feat(lower): control PID tune mode with board keys  # 加物理按键控制
0414ee0 fix(lower): use legacy key pins for PID tune mode   # 按键改成 legacy 引脚
e1a94f6 tune(lower): raise default speed target             # 提高默认目标
76a6381 tune(lower): set speed target floor                 # 设目标速度下限
842800b debug(lower): expose encoder window telemetry       # 暴露窗口遥测
830d91d debug(lower): print raw encoder speed               # 打印 2ms 原始增量
a1fbd34 fix(lower): prevent reverse output in speed tune    # 单向输出，消除反转抽动
43424ea fix(lower): accept idle uart tune commands          # 无换行命令空闲自动提交
7f6d789 tune(lower): persist stable low speed defaults      # 写入稳定默认 target=80/p6/i0.3
618591e refactor(lower): clarify tune duty helpers          # 拆分死区/限幅函数，修注释
04c0921 tune(lower): use longer encoder speed window        # 窗口加长到 200ms
b59a96f tune(lower): raise stable low speed integral gain   # 默认 Ki 提到 0.5
3cdc972 chore: ignore local tool outputs                   # .gitignore + 移除 uvoptx
43cfea4 fix(lower): default to executor mode for dual board test # 默认 LOWER_PID_TUNE=0
```

### 5.2 上板 `LHX/upper-test`（最近）
```
0bdc88c fix(upper): lock fixed target speed and disable wheel balance for bench test # 架空固定目标80 cnt/s + 禁用 wheel_balance
259176a fix(upper): make closed loop bench test forward only # 架空速度闭环：禁位置环 + 禁反向
cea469c test(upper): switch back to closed loop mode       # OPENLOOP_TEST_ENABLE=0，进入闭环架空验证
80322ba fix(upper): do not clamp open loop duties to PID cap # 修复 1200/1500/1800 被 ClampMotorDuty 截成 1000
050c9b1 fix(upper): keep open loop OLED page stable        # 开环模式跳过普通遥测页，避免 OLED 跳变
7aaf485 tune(upper): use conservative speed PID baseline   # 速度 PID 起点改 6/0.5/0
defa660 test(upper): make open loop key check immediate    # 按下即生效 + OLED raw 按键/duty
9a1e29a test(upper): raise open loop check duties          # 开环占空比提到 1200/1500/1800
ad2399f test(upper): enable open loop motor check          # 开环测试开关置 1
856109d chore: ignore local tool outputs                   # 同步忽略规则
```
（更早还有编码器单位修正、陀螺标定、降速等提交：`69266ac`、`72ddf75`、`9f74e00`、`436b719`、`3b3ca36`。）

### 5.3 仓库整理提交 `3cdc972`（已同步到所有分支）
通过 cherry-pick 到每条分支：新增 `.gitignore` 忽略 `image_reader/`、`video_reader/`，并把已被跟踪的 `Project.uvoptx` 从版本控制移除（本地文件保留）。各分支落地的提交哈希：
```
LHX/competition  6814f21      LHX/legacy-motor f7eb228
LHX/lower        ee6cd37      LHX/lower-pid    3cdc972
LHX/pcb2         16097b6      LHX/sensor       f2c64ab
LHX/upper        05e3367      LHX/upper-test   856109d
version-0        246a8a9
```
注：`competition / legacy-motor / sensor / version-0` 当前显示 `ahead 1`，本地领先远端一次提交，需要时再 push。

### 5.4 关键修复速查
- 反转抽动：`a1fbd34` 速度环单向前进，负输出钳到 0。
- `t80` 不生效：`43424ea` 串口空闲 20ms 自动提交。
- 显示分辨率粗：`04c0921` 窗口 100ms→200ms（1 tick 由 10 cnt/s 变 5 cnt/s）。
- 默认参数固化：`7f6d789` + `b59a96f`，目标 80、p6/i0.5/d0。
- 开环不动：`9a1e29a` 占空比提高跨过死区。

### 5.5 常用 git 命令备查
```
git -C <repo> status -sb                 # 看当前分支与改动
git -C <repo> diff --check               # 提交前检查空白/冲突标记
git -C <repo> log --oneline -n 14 <分支>  # 看某分支提交历史
git -C <repo> branch -vv                 # 看所有分支与远端跟踪状态
```

## 6. 重要结论 / 约束
- 不能用提高目标速度来换取测速精度，电机有烧毁风险。低速调参只能从测速窗口、滤波、显示口径改善。
- 编码器硬件正常：dt 稳定、tick 对应速度，不是编码器坏。
- 想“一圈 100 变 1000”这种真实分辨率提升只能靠硬件（更高 PPR 编码器 / 装电机轴侧 / 排查漏边沿），软件插值不增加真实位置信息。
- 下板调参参数 `p6 i0.5 d0` 只在 `LOWER_PID_TUNE=1` 生效；恢复双层后真正速度 PID 在上板。
- 架空验证阶段：`BENCH_FIXED_SPEED_ENABLE=1` 固定目标 80 cnt/s，跳过 Path 状态机；`wheel_balance` 强制关。上路前必须改回 0。
- 上路前必须把 `BENCH_FIXED_SPEED_ENABLE` 改回 0（`User/PID_Controller.c`），否则循迹没有速度规划，所有路段按固定 80 cnt/s 跑。

## 7. 上板速度环口径（upper-test）
- 反馈已是 cnt/s 口径（与下板一致）。
- 当前速度 PID 起点已改成与下板低速调稳值一致：
```c
SpeedPID_Init(&g_speed_pid, 6.0f, 0.5f, 0.0f, 8000.0f, -8000.0f);
```
- 注意：当前 `OPENLOOP_TEST_ENABLE=1` 时不跑速度 PID；只有切回 `0` 后该参数才参与闭环。

## 8. 上板按钮作用
正常循迹模式（`OPENLOOP_TEST_ENABLE = 0`）：
- K1 启动运行
- K2 停止运行
- K3 停止 + 复位丢线/位置
- K4 仅显示

开环测试模式（`OPENLOOP_TEST_ENABLE = 1`，当前已关闭；需要排查链路时再打开）：
- K1 = 1200（12%）
- K2 = 1500（15%）
- K3 = 1800（18%）
- K4 停止
- 当前开环按键为 raw 状态读取：按下即生效，不再等松开事件。
- OLED 开环模式使用独立稳定页面，不再调用普通 `TelemetryScreen_Update()`：
  - 第 1 行：`OPENLOOP RUN/STOP`
  - 第 2 行：`K:1234 D:+1200`（raw 按键位 + 当前 duty）
  - 第 3 行：`L:+xxx R:+xxx`（左右轮速度）
  - 第 4 行：`LINK:OK A:1`（下板心跳 + 开环 active）

上板按键引脚：K1=PB14，K2=PB13，K3=PC14，K4=PC13。

## 9. K1/K2/K3 无反应的判断
原原因之一：开环占空比 500/800/1000 低于电机起步死区（左右约 750/950），可能完全不动。已提高到 1200/1500/1800。
已新增诊断：开环按键改为按下即生效，OLED 第 2 行显示 raw 按键位和当前 duty；`050c9b1` 后页面稳定，不应再跳变。
- 第 2 行 `K:` 按下后没有对应数字 → 上板按键扫描/引脚问题。
- 第 2 行 `K:` 有数字且 `D:` 变为 1200/1500/1800，但第 3 行 L/R 一直 0、轮子不动 → 下板未处于执行器模式、板间串口/协议问题，或电机供电/使能问题。
- 曾出现 `D:` 随 K1/K2/K3 变化但实际轮速都一样：根因是开环发送前误用 `ClampMotorDuty()`，被 `MOTOR_DUTY_HARD_CAP=1000` 截成同一个命令。`80322ba` 已改为 `ClampMotorDutyFinal()`。
- 第 3 行 L/R 变化但轮子肉眼不明显 → 编码器链路通，优先看电机供电、机械负载、占空比是否被硬钳/死区影响。
- 第 3 行 L/R 随 K1/K2/K3 阶梯增加且轮子转 → 开环链路通过，可以切回闭环。

## 10. 下一步测试流程
1. 下板烧最新 `LHX/lower-pid`，默认 `LOWER_PID_TUNE = 0`（纯执行器：收 MOTOR_CMD、回传 ENC_FEEDBACK、看门狗）。
2. 上板烧最新 `LHX/upper-test`（当前 `OPENLOOP_TEST_ENABLE = 0`，闭环；`BENCH_FIXED_SPEED_ENABLE = 1`，固定 80 cnt/s；`POSITION_LOOP_ENABLE=0`）。
3. 架空轮子，按 K1 启动，观察左右轮速度是否平稳保持在 ~80 cnt/s 左右，不再持续加速。
4. 按 K2 停止，确认马上停车；断开上板发送约 200ms 内下板应 watchdog 停车。
5. 架空速度闭环稳定后，把 `BENCH_FIXED_SPEED_ENABLE` 改回 0 → 再打开 `POSITION_LOOP_ENABLE=1` 做架空循迹闭环。
6. 最后才落地最低速短跑。**上路前务必确认 `BENCH_FIXED_SPEED_ENABLE=0`，否则没有循迹速度规划。**

## 11. 分支整理建议
- 保留 `LHX/lower-pid` 作为下板主分支（含纯执行器模式 + 调试上下文）。
- `LHX/lower` 暂留，双层验证通过后再删，避免烧错旧分支。

## 12. 关键代码片段（便于脱离仓库复查）
文件：`User/main.c`（下板 `LHX/lower-pid`，`LOWER_PID_TUNE=1` 段）

下板调参常量：
```c
#define TUNE_SPEED_WIN_MS    200u     // 1 tick = 5 cnt/s
#define TUNE_HARD_CAP        1000.0f  // PID 调节量限幅
#define TUNE_DEADZONE_L      750.0f   // 左轮起步死区
#define TUNE_DEADZONE_R      950.0f   // 右轮起步死区
#define TUNE_FINAL_CAP       (TUNE_HARD_CAP + TUNE_DEADZONE_R) // 1950 安全上限
#define TUNE_PRINT_MS        200u
#define TUNE_UART_IDLE_MS    20u      // 无换行命令空闲自动提交
#define TUNE_MIN_TARGET      70.0f
#define TUNE_DEFAULT_TARGET  80.0f
#define TUNE_TARGET_STEP     10.0f
```

单向输出 + 死区前馈：
```c
static void tune_apply_motor(uint8_t motor_id, float duty_cmd)
{
    uint16_t duty;
    if (duty_cmd < 0.0f) duty_cmd = 0.0f;                 // 负输出钳 0，永不反向
    if (duty_cmd > (float)MOTOR_DUTY_MAX) duty_cmd = (float)MOTOR_DUTY_MAX;
    duty = (uint16_t)duty_cmd;
    Motor_SetDirection(motor_id, MOTOR_DIR_FORWARD);      // 调参恒前进
    Motor_SetSpeed(motor_id, duty);
}

static float tune_wheel_duty(float duty_cmd, float deadzone)
{
    return tune_clamp_forward(
        tune_add_forward_deadzone(tune_clamp_forward(duty_cmd, TUNE_HARD_CAP), deadzone),
        TUNE_FINAL_CAP);
}
```

串口空闲自动提交（修 `t80` 不生效）：
```c
now = Millis_Get();
if (!got_byte && idx > 0u && (uint32_t)(now - last_rx_ms) >= TUNE_UART_IDLE_MS)
{
    buf[idx] = '\0';
    tune_handle_line(buf, p_target, p_run);   // 没换行也提交
    idx = 0u;
}
```

默认 PID 初始化：
```c
tune_pid_init(&g_tune_pid, 6.0f, 0.5f, 0.0f, TUNE_HARD_CAP, 0.0f);
```

文件：`User/main.c`（上板 `LHX/upper-test`）

开环测试占空比：
```c
#define OPENLOOP_TEST_ENABLE 1
#define OPENLOOP_DUTY_12PCT  1200  // 跨过左右电机起步死区
#define OPENLOOP_DUTY_15PCT  1500
#define OPENLOOP_DUTY_18PCT  1800
```

开环下发（跳过 PID，直接发固定占空比）：
```c
int16_t duty = (int16_t)ClampMotorDutyFinal((float)g_openloop_duty);
Proto_SendMotorCmd(duty, duty, g_openloop_active);
```
注意：开环 duty 是绝对占空比，不走 PID 调节量上限 `ClampMotorDuty()`，否则 1200/1500/1800 会全部被钳到 1000。

闭环架空测试保护（`259176a` 后）：
```c
#define POSITION_LOOP_ENABLE 0
#define CLOSED_LOOP_REVERSE_ENABLE 0
```
结论：当前闭环只验证速度环，位置环不参与；任何负向输出都会被钳到 0，避免下板真反转导致前后抽动。速度环稳定后，再打开位置环。

架空固定速度目标（`0bdc88c` 后，`User/PID_Controller.c`）：
```c
/* 架空台架速度闭环验证：1=固定速度目标，跳过 Path 状态机。
 * 原因：架空时 total_dist_cm 由编码器假累计推得飞快，Path 状态机会一路升档
 *      (SEARCH 90 -> STRAIGHT 140 -> LINE_FOLLOW 115 ...)，
 *      表现为"目标速度被一直推高"导致单向加速。
 * 上路前必须置 0，否则没有循迹速度规划。 */
#ifndef BENCH_FIXED_SPEED_ENABLE
#define BENCH_FIXED_SPEED_ENABLE 1
#endif
#ifndef BENCH_FIXED_TARGET_CPS
#define BENCH_FIXED_TARGET_CPS 80.0f
#endif
```
架空模式下：
- 速度目标固定 80 cnt/s，不再查 `Path_GetTargetSpeed()`。
- `wheel_balance` 强制 0，避免架空左右速度差被 `WHEEL_BALANCE_KP=2.0` 放大成 ±300 差分输出。
- 上路前必须把 `BENCH_FIXED_SPEED_ENABLE` 改回 0，恢复正常循迹速度规划。

开环 OLED 稳定页面（`050c9b1` 后）：
```c
#if OPENLOOP_TEST_ENABLE
    OpenLoop_ShowStatus();
#else
    TelemetryScreen_Update();
#endif
```
结论：开环模式只画开环诊断页，避免普通遥测页与诊断页轮流清行导致 OLED 跳变。

## 13. 板间协议速记
- `0x20 MOTOR_CMD`（上→下）：`[duty_l_hi][duty_l_lo][duty_r_hi][duty_r_lo][flags]`，duty 为带符号占空比 -10000~+10000，flags bit0=使能。
- `0x21 ENC_FEEDBACK`（下→上）：`[spd_l][spd_r][cnt_l 4B][cnt_r 4B]`，spd 为每周期增量，cnt 为累计计数（大端）。
- `0x22 LINK_RESET`：开机/重烧通告对端复位到安全态。
- 下板看门狗：`LOWER_PID_TUNE=0` 时，上板停发 MOTOR_CMD 超时约 200ms 自动停车下电。
- 上板心跳监视：连续约 100ms 收不到下板心跳判为断链，运行中会停车防窜。

## 14. 硬件情况
板型：二合一板，上下两块 STM32F10x，USART2 作板间链路（115200，二进制帧）。

### 14.1 下板（执行器）引脚
电机（DRV8701，TIM1，PWM 17KHz）：
| 功能 | 引脚 | 说明 |
| --- | --- | --- |
| 电机使能 | PA12 | 高=使能，低=下电 |
| 电机R 方向 | PA8 | MOTOR_R=电机1 |
| 电机R PWM | PA9 | TIM1 CH2 |
| 电机L 方向 | PA10 | MOTOR_L=电机2 |
| 电机L PWM | PA11 | TIM1 CH4 |

编码器（编码器模式，TI12 上升沿）：
| 功能 | 定时器 | 引脚 |
| --- | --- | --- |
| 左轮 | TIM4 | PB6 / PB7 |
| 右轮 | TIM3 | PA6 / PA7（读数取负） |

下板按键（低电平有效，上拉输入）：K1=PA5，K2=PA4，K3=PC14，K4=PC13。

### 14.2 上板（大脑）外设
- 7 路灰度循迹传感器（黑线识别 → 位置）。
- MPU6050（陀螺，带静止标定 + Z 轴零偏跟踪）。
- OLED 显示。
- RGB 指示灯。
- 物理按键：K1=PB14，K2=PB13，K3=PC14，K4=PC13。
- 电池电压采样 PA0。

注意：上板和下板的按键引脚不同（上板 K1/K2 在 PB14/PB13，下板 K1/K2 在 PA5/PA4），别混。

### 14.3 关键硬件参数与安全限制
```c
MOTOR_FREQUENCY_17KHZ = 17000   // PWM 频率
MOTOR_DUTY_MAX        = 10000   // 满量程占空比
MOTOR_DUTY_SAFE_MAX   = 2000    // 下板硬钳上限：Motor_SetSpeed 超过即截到 2000
TIM1: PSC=0, ARR=4234           // 72MHz/4235 ≈ 17KHz
```
- **下板 `MOTOR_DUTY_SAFE_MAX=2000` 是硬保护**：任何占空比（含上板开环 K3=1800）经 `Motor_SetSpeed` 都会被钳到 ≤2000，电机不会全速。这也是低速安全的最后一道闸。
- 电机起步死区实测：左轮约 750，右轮约 950（满量程 10000 口径）。低于死区电机不动。
- 开机默认两电机方向 FORWARD，初始 `Motor_Disable` + `Motor_StopAll`。

### 14.4 已确认的硬件健康状态
- 编码器硬件正常：与 `LHX/legacy-motor` 初始化一致，dt 稳定、tick 随速度变化，2ms RAW 在低速为 0/1 属正常。
- 电机方向逻辑：调参模式恒前进，无反转抽动（软件保证）。
- 真实编码器分辨率受限于硬件 PPR / 安装位置，软件无法提升真实计数分辨率。
