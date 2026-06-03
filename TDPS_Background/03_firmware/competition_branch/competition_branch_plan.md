# TDPS 竞赛任务分支说明

## 分支定位

当前项目分为两类主要开发分支：

- **`LHX/sensor`**：硬件测试、传感器测试、安全锁测试分支。默认用于确认 7 路光电、IMU、编码器、电池采样、OLED/串口遥测等基础硬件是否正常。
- **`LHX/competition`**：TDPS 规定任务竞赛分支。用于沉淀 Task 1 巡线、Task 2 通信、Task 3 雷达相关的完整流程代码。

后续硬件测试中发现的问题，应先在 `LHX/sensor` 上用安全测试代码复现和修正；确认问题解决后，再把对应修正同步到 `LHX/competition`，避免竞赛分支长期落后于实测硬件状态。

## 当前竞赛分支安全状态

`LHX/competition` 已加入完整流程入口，但默认仍保持安全：

- `TDPS_COMPETITION_ENABLE = 0` 时，不允许电机闭环输出。
- 每个控制周期仍会采样光电板并更新黑线位置。
- 每个控制周期会调用安全停机逻辑，确保电机和风扇保持关闭。
- 风扇继续使用 `FanMotor_SafetyLock`，当前不输出 PWM。

只有在确认硬件、电机驱动、电池、电流和机械结构安全后，才允许把 `TDPS_COMPETITION_ENABLE` 改为 `1` 做低速竞赛流程测试。

## 竞赛流程入口

当 `TDPS_COMPETITION_ENABLE = 1` 时：

- **K1**：启动竞赛巡线流程。
  - 复位黑线位置历史。
  - 清零丢线计数。
  - 关闭风扇。
  - 调用 `Path_StartRace()`。
  - 调用 `Motor_Enable()`。
  - 设置 `is_racing = 1`。
- **K2**：立即停止全部运动。
- **K3**：运行中强制切回普通巡线段 `SEG_LINE_FOLLOW`，用于现场恢复/调试。
- **K4**：运行中强制切到 `SEG_RADAR_APPROACH`，用于 Task 3 相关调试入口。

主控制周期中：

1. `LineSensor_SampleAll()` 采样 7 路光电和 PA0 电池电压。
2. `BlackPoint_Finder_Search()` 计算黑线位置。
3. 找到黑线时更新 `position_get`。
4. 丢线超过 `TDPS_COMPETITION_LINE_LOST_LIMIT` 后立即 `SafetyStopAll()`。
5. `is_racing = 1` 时执行 `PID_Control_Update()`。
6. `Path_GetCurrentSegment() == SEG_FINISH` 后立即停车。

## 7 路独立光电传感器算法

当前新光电板与旧 16 路 74HC4067 多路复用方案不同。新方案按 7 路独立输入处理，不再按旧通道串扰模型设计算法。

当前识别链路为：

- `LineSensor.c` 将 7 路数字输入映射为伪 ADC 值：
  - 黑线：`0`
  - 白底：`4095`
- `BlackPoint_Finder_IsBlackPoint()` 判断某一路是否压黑线。
- `BlackPoint_Finder_Search()` 构造 7-bit 黑线掩码。
- 对掩码中的连续黑线段求中心。
- 若出现多段黑线，选择距离上一次位置最近的连续段。
- 若完全丢线，保持上一次位置并返回 `found = 0`。

这种逻辑适合独立 7 路数字光电板，比旧的模拟归一化最小值算法更适合当前硬件。

## 当前地图状态机状态

`Path.c` 已保留 Patio 赛道的完整任务段枚举：

- `SEG_START_SEARCH`
- `SEG_START_STRAIGHT`
- `SEG_LINE_FOLLOW`
- `SEG_U_TURN`
- `SEG_S_CURVE`
- `SEG_BOX_1`
- `SEG_QUAD_CIRCLES`
- `SEG_RADAR_APPROACH`
- `SEG_FINISH_APPROACH`
- `SEG_FINISH`

当前状态机仍属于基础可运行骨架，依赖编码器里程、弯道强度和丢线状态切换路段。真实上赛道前必须重新标定：

- 编码器每厘米脉冲数 `ENCODER_TICKS_PER_CM`。
- 各路段距离阈值。
- 弯道强度阈值。
- 方框/圆形干扰区策略。
- 起点、marker 和终点停车策略。

## 当前低速安全策略

竞赛分支继续遵守当前电机保护边界：

- `MOTOR_DUTY_SAFE_MAX = 2000`。
- PID 速度环和位置环输出限幅低于该安全占空比上限。
- `Path.c` 的速度目标单位为 2ms 控制周期内编码器增量，不再按 PWM 占空比解释。
- 风扇保持 `FAN_MOTOR_SAFE_DUTY_OFF = 0`。

## 后续同步规则

建议后续按以下流程维护：

1. 在 `LHX/sensor` 做硬件测试。
2. 如果发现传感器、电机、编码器、IMU 或通信问题，先在 `LHX/sensor` 修复并验证。
3. 将确认有效的修正同步到 `LHX/competition`。
4. 在 `TDPS_Background` 记录测试现象、修正原因和同步状态。
5. 竞赛分支只在确认安全后开启 `TDPS_COMPETITION_ENABLE = 1` 做低速闭环测试。
