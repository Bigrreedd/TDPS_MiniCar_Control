### Patio 赛道策略与控制建议（基于当前微缩车模平台）

本文件结合 `L1b_Design-Tasks_An-Overview_2025-2026` 课件给出的 **Patio 赛道尺寸**（总长 800 cm、宽 500 cm、起终点 50×50 cm、直线与弯道、箱体区域以及 2.1/2.2 无线通信门、雷达箱体等），对现有 `微缩车模` 控制程序给出：

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
  - 当前 `BlackPoint_Finder_Search()` 输出的 `precise_position` 可以视为 \[0, 15\] 范围的“线中心位置”。

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
  - 视觉上，黑线整体相对于 Patio 全局坐标是镜像，但对小车本体坐标系而言，只要始终“在线中心附近”，控制逻辑无需区分“左赛道/右赛道”；
  - 真正需要区分的只是 **LoRa 触发的顺序和雷达箱体前的路径选择方向**。

- **识别方法建议**：
  1. 利用**开局第一个明显大弯道的方向**来区分：
     - 统计在前若干秒内 `position_get` 的偏差符号：如果总体偏右（`position_get > target`），说明黑线更多出现在右侧，对应一侧赛道；反之偏左对应另一侧。
  2. 一旦判定了“赛道类型”，设置一个枚举 `track_side = LEFT_TRACK / RIGHT_TRACK` 存入全局，用于后续：
     - 预判 LoRa 门所在的大致距离/时间窗口；
     - 预判 Task 3 时“箱体中自由侧”的默认方向（例如左赛道默认先扫描右侧箱壁，右赛道默认先扫描左侧箱壁），减少扫描时间。

---

### 二、赛道路段建模与速度自适应策略

结合 Patio 地图尺寸，可将一条赛道划分为若干**特征路段**：

1. **起跑直线**（约 70 cm）
2. **大 S 弯**（约 200 cm 高度内的曲线段）
3. **箱体/窄道组合区域**（多处 50 cm 宽的箱体/门、障碍区和 30–50 cm 的过渡直线）
4. **LoRa 门（2.1 / 2.2）附近的直线 + 箱体**
5. **Task 3 雷达箱体前的直线与箱体**
6. **终点前最后一段 S 弯和直线**

#### 1. 利用“黑线几何特征 + 速度反馈”识别路段

不依赖绝对位置传感器，仅靠当前已有传感器就可以做**弱建模**：

- 利用 `position_get` 的**一阶差分/二阶差分**判断弯道强度：
  - 弯道处 \|`position_get - target_position`\| 较大，且导数变化较快；
  - 直线处偏差绝对值和变化率都较小。
- 利用编码器积分估计**行驶距离**：
  - 将左右轮脉冲和转换为近似行驶距离，结合赛道标称长度对齐不同路段的“里程区间”；
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
  - 根据比赛策略可选择**略微提高速度**以冲线，但应增加“终点停车”逻辑：
    - 利用线末端的几何特征（如终点框内线段终止）+ 里程估计，在接近终点时降低速度并在 Finish 红框内缓慢停止。

#### 3. 多圆“误导区”最短路径（直上 + 只走顶半圆）

你指出的这段（地图上靠近顶部、四个圆形堆叠的区域）确实是**光电阵列最容易被“分叉/环线”误导**的地方：如果单纯“取全局最黑点”跟随，车很容易钻进下面的圆环线路径。这里建议把该区域当作**有策略的特殊赛段**，明确分成两步：

- **步骤 A：直上（不进下方圆环）**  
  目标是沿着主干竖直线一路到最上方，只允许小幅修正，避免“跳线”到下方圆环。
- **步骤 B：顶半圆（只绕最上方那一个圆的上半圈）**  
  到顶部后执行一次**受约束的半圆转向**，把车从竖直方向“接”到上方水平连线（朝中间 Finish 方向）。

下面给出“左/右镜像赛道”两种情况下的**光电识别规则**与**左右轮差速（电机运动方向）控制要点**。

##### 3.1 左/右赛道的转向方向（镜像关系）

- **左侧赛道（左边 Start 出发）**：多圆在主干线**右侧**，顶半圆连接到上方水平线时需要整体**右转**（顺时针弧线）。
- **右侧赛道（右边 Start 出发）**：多圆在主干线**左侧**，顶半圆连接到上方水平线时需要整体**左转**（逆时针弧线）。

可用一个统一变量表示：

```c
turn_dir = (track_side == LEFT_TRACK) ? TURN_RIGHT : TURN_LEFT;
// TURN_RIGHT => 左轮更快、右轮更慢；TURN_LEFT 反之
```

##### 3.2 光电阵列如何“识别自己被误导了”

在多圆区，常见的误导现象是：阵列同一时刻“看到”不止一条黑线（例如主干线 + 圆环线），表现为：

- **多黑点同时出现**：归一化后很黑（接近 0）的通道数量明显增多；
- **左右两侧同时很黑**：左半阵列和右半阵列都出现很小的归一化值；
- **最黑点跳变**：`precise_position` 在很短时间内从靠中间/一侧跳到另一侧（跳变幅度大且频繁）。

建议在 `BlackPoint_Finder_Search()` 的结果之上加一个“分叉/环线检测”：

```c
bool is_confusing_zone = (black_count >= 3) || (left_has_black && right_has_black) || (jump_too_fast);
```

其中 `black_count` 可以用“归一化值 < 阈值”的通道数近似（阈值如 0.25），`jump_too_fast` 用最近 5~10 次 `precise_position` 的变化量/方差判断。

##### 3.3 步骤 A：直上阶段——“锁定主干线，不被下方圆环拉走”

直上阶段的核心不是追最黑，而是追“**与上一次主干位置一致**”：

- **规则 1（连续性优先）**：若检测到 `is_confusing_zone`，则不要切换到新的全局最黑点；改为选择**更靠近 `last_precise_position`** 的候选线（“就近跟随”）。
- **规则 2（远离圆环侧）**：在 `is_confusing_zone` 下，对 `precise_position` 施加一个小偏置，把车朝“远离圆环的一侧”压一点，以减少圆环线对阵列的覆盖概率。  
  - 左侧赛道（圆在右）：偏置向左（减小 `precise_position`）  
  - 右侧赛道（圆在左）：偏置向右（增大 `precise_position`）
- **速度规则**：直上阶段建议中低速（例如将 `i_speed` 上限限制在直线路段的 60%~75%），确保纠偏时不会甩进圆环。

##### 3.4 步骤 B：顶半圆阶段——“明确的左右轮差速 + 受约束的跟线”

顶半圆建议用“**约束转向**”来保证只走上半圈，不走下半圈：

- **控制目标**：在一小段时间/里程内保持“固定转向方向”，同时允许光电提供微调，但**禁止转向方向反复翻转**。
- **电机差速要点**（以恒定前进为主，不倒车）：
  - **右转（TURN_RIGHT）**：`left_speed > right_speed`（左轮快、右轮慢）
  - **左转（TURN_LEFT）**：`right_speed > left_speed`

结合你们现有输出形式（`left_output = base + corr; right_output = base - corr;`）可以直接记住：

- **`corr > 0` 产生右转**（左更快右更慢）
- **`corr < 0` 产生左转**

因此在顶半圆阶段可做“符号约束 + 限幅”：

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

**顶半圆结束判据**建议用“里程 + 光电稳定性”双条件（比纯时间更稳）：

- **里程**：从进入顶半圆开始累计行驶距离达到预设（以地图尺寸估算后实测微调；该半圆直径标注约 20 cm，则半圆弧长约 \( \pi \times 10 \) cm ≈ 31 cm；考虑进出连接段，可把目标里程设为 40–60 cm 再标定）。
- **光电稳定**：当 `precise_position` 回到“上方水平主干线”的稳定区间（例如接近阵列中间），且 `is_confusing_zone == false` 持续 \(N\) 次，则退出顶半圆模式，回到正常循迹 PID。

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
    - 若 \|`gz_rads`\| 超过阈值（表示急转弯或打滑），则暂时降低速度环目标速度，并减弱位置环 `Kp`，防止过度修正。

- **丢线补偿**：
  - 利用 `last_precise_position`，在 `found == 0` 且时间未过长时，继续按“上一次线方向”进行小幅控制，并快速减速；
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
  - 在进入该区间时，开启“LoRa 预警窗口”：一旦检测到黑线前端进入**门框箱体的直线区域**（偏差较稳定，箱体两侧障碍线条特征），则在 0.5 s 内发送一次 LoRa 数据；
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

**在 gate 处发送数据帧**（假设我们采用简单的 ASCII 文本协议，上层由拱门接收端解析）：

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
2. 在距离箱体入口约 30–50 cm 处减速进入“雷达扫描模式”：
   - 固定低速直行，保持在黑线中心；
   - 连续 \(N\) 次读取左右雷达的目标距离（或反射强度）；
3. 判断哪一侧存在障碍：
   - 例如：左雷达最近目标距离 \(d_L < 60\) cm 且回波强度高，而右雷达无明显目标或距离 \(d_R > 80\) cm，则认为**障碍在左侧**；
4. 选择**相反侧**作为通道：在进入箱体时偏向无障碍一侧，并在箱体内部保持较小但固定的侧向偏移（类似“沿箱壁走”）。

#### 3. 基于标准库的伪代码示例

假设雷达模块通过 UART 或 SPI 提供**距离数据**，我们以 UART 为例（类似 LoRa）：

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

1. **根据赛道模型做“速度规划曲线”而不是简单的 if-else**  
   - 利用里程估计，将赛道分段后为每一段预设目标速度曲线（类似赛车的“赛道地图”），再叠加当前偏差/IMU 的反馈动态缩放；
   - 这比单纯根据瞬时偏差降速更平滑、更可预测。

2. **使用模型预测控制（MPC）或前瞻控制**  
   - 在现有双环 PID 的基础上加入**预瞻距离**概念，例如根据当前偏差和其导数预测若干采样周期后的偏差，将预测值作为控制输入；
   - 可以大幅提升高速弯道下的稳定性，减小超调。

3. **联合雷达与光电实现“障碍感知 + 轨迹重规划”**  
   - 当前 Task 3 只在箱体中使用雷达，实际上可以在整条赛道上用雷达检测前方大障碍（误入他队车道、工作人员进入赛道等），并执行紧急刹车或绕行；
   - 通过将雷达距离作为额外约束输入到速度规划中（例如前车距 < 某阈值时自动限速）。

4. **数据记录与离线分析**  
   - 使用串口或外接存储（如 SD 卡）定期记录关键数据：`position_get`、`speed_left/right`、`gz_rads`、`i_speed`、赛段 ID 等；
   - 赛后在 PC 上进行可视化分析（Matlab/Python），针对**每个弯道和箱体**寻找最佳 PID 参数与速度上限。

5. **参数自标定与赛前自检流程**  
   - 在比赛前加入一键“自检与标定”：  
     - 在固定白纸/黑线环境下自动扫描 16 路光电，重新估计 `min/max` 与阈值；  
     - 简单直线加减速测试，自动测量最大安全加速度与制动距离；  
   - 将结果保存到 Flash 或 EEPROM 中，使得在不同光照与地板条件下都能快速适应。

6. **软件结构化与模式管理**  
   - 将整车运行抽象为有限状态机（FSM），例如 `IDLE / START_SEARCH / LINE_FOLLOW / LORA_GATE / RADAR_SCAN / BOX_PASS / FINISH / EMERGENCY_STOP`；
   - 使得不同任务（Task 1/2/3）在代码结构上清晰解耦，也方便调试与评分展示。

---

以上建议与伪代码均基于当前 `微缩车模` 工程的**标准外设库结构**和已有模块（电机、光电阵列、IMU、PID 控制等）设计，实际实现时可逐步迭代：先搭建状态机与里程估计框架，再逐段调参与验证，以尽量在有限时间内让小车在 Patio 赛道上实现**稳定、自主、快速**的整体表现。

