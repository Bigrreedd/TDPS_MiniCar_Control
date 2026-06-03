#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "LineSensor.h"
#include "Uart_Config.h"
#include "stdio.h"
#include "math.h"
#include "RGB_Led.h"
#include "MPU6050_Config.h"
#include "pose.h"
#include "Key_Scan.h"
#include "system_stm32f10x.h"
#include "BlackPoint_Finder.h"
#include "TelemetryScreen.h"
#include "Protocol.h"
#include "Battery.h"
#include "ABEncoder.h"
#include "PID_Controller.h"
#include "Path.h"
#include "Motor_ctr.h"
#include "stm32f10x_it.h"

/*
 * ============================================================
 *  二合一板 —— 上板（大脑）固件
 *  职责：7 路灰度循迹 + MPU6050 + OLED + 按键启动，
 *        本地运行全部控制算法：位置环 + 速度环 + Path
 *        + 15% 起步 + 丢线保护，最终通过 USART2 下发
 *        MOTOR_CMD（左/右带符号占空比 + 使能）给下板。
 *  轮速反馈来自下板的 ENC_FEEDBACK 帧。
 *  设计目标：调试时只烧上板，下板（纯执行器）烧一次永不变。
 *  注意：USART2 是“板间链路”，只发二进制帧，不输出调试文本。
 * ============================================================
 */

// 滴答定时器初始化，2ms中断一次 (500Hz)
void SysTick_Init(void)
{
    SysTick_Config(SystemCoreClock / 500);
}

float BDI_V = 0;
volatile uint8_t is_racing = 0;            /* 按键启动标志：1=运行，发送给下板 */
static uint8_t g_imu_init_ok = 0;
static uint8_t g_imu_who_id = 0;

// 控制环状态（从 SysTick 移到主循环）
// position_get 的 extern 声明已在 stm32f10x_it.h 中
BlackPointResult_t result_BlackPoint;
static uint32_t lose_time = 0;

// ESP32-S3 雷达数据（保留协议兼容）
volatile uint16_t radar_distance_cm = 0;

/* 起步时给速度环输出的初值(满量程 10000)。
 * 死区前馈已负责克服起步静摩擦，这里只需给速度环一个小的正向初值避免从 0 慢爬，
 * 取 200；过大会和死区前馈叠加导致起步窜车。 */
#define START_DUTY_15PCT  200.0f

/* 电机占空比硬上限（满量程 10000）：钳住 PID 速度环的调节量(不含死区前馈)。
 * 1000 = 满量程的 10%。叠加右轮最大死区 950 后最终≈1950，仍 <下板 SAFE_MAX(2000)，
 * 故下板安全限无需改动。调速就改这一个值：想更慢往下调，想更快往上调(但需留出死区余量)。 */
#ifndef MOTOR_DUTY_HARD_CAP
#define MOTOR_DUTY_HARD_CAP  1000.0f
#endif

/* 带符号占空比限幅到 [-CAP, +CAP]，保留方向 */
static float ClampMotorDuty(float duty)
{
    if (duty >  MOTOR_DUTY_HARD_CAP) return  MOTOR_DUTY_HARD_CAP;
    if (duty < -MOTOR_DUTY_HARD_CAP) return -MOTOR_DUTY_HARD_CAP;
    return duty;
}

/* 电机起步死区前馈：实测右轮约需 10%(1000) 才起步、左轮约 8%(800)，
 * 死区以下电机不动，PID 输出落在死区内等于白调。
 * 故对带方向的非零命令垫上一个起步基值，把 PID 的工作点抬到电机能动的区间，
 * 左右分别配置以补偿起步摩擦不对称(右轮更难起步)。
 * 命令绝对值低于 EPS 视为停车(返回 0)，避免 0 附近抖动与静止蠕行。
 * 上车微调：某轮起步偏迟就调高对应 DEADZONE；起步窜得猛就调低。 */
#ifndef MOTOR_DEADZONE_L
#define MOTOR_DEADZONE_L  750.0f
#endif
#ifndef MOTOR_DEADZONE_R
#define MOTOR_DEADZONE_R  950.0f
#endif
#ifndef MOTOR_CMD_EPS
#define MOTOR_CMD_EPS     1.0f
#endif
#ifndef CLOSED_LOOP_REVERSE_ENABLE
#define CLOSED_LOOP_REVERSE_ENABLE 0
#endif

static float ApplyDeadzone(float duty, float deadzone)
{
    if (duty >  MOTOR_CMD_EPS) return duty + deadzone;
    if (duty < -MOTOR_CMD_EPS) return duty - deadzone;
    return 0.0f;   /* 近零命令直接停，不蠕行 */
}

static float ClampClosedLoopDuty(float duty)
{
    duty = ClampMotorDuty(duty);
#if !CLOSED_LOOP_REVERSE_ENABLE
    if (duty < MOTOR_CMD_EPS) return 0.0f;
#endif
    return duty;
}

/* 死区前馈后的最终安全上限：= PID 上限 + 最大死区，使 PID 满输出叠加死区后不被砍。
 * 仍兜一个绝对天花板防止异常值窜车。 */
#ifndef MOTOR_DUTY_FINAL_CAP
#define MOTOR_DUTY_FINAL_CAP  (MOTOR_DUTY_HARD_CAP + MOTOR_DEADZONE_R)
#endif
static float ClampMotorDutyFinal(float duty)
{
    if (duty >  MOTOR_DUTY_FINAL_CAP) return  MOTOR_DUTY_FINAL_CAP;
    if (duty < -MOTOR_DUTY_FINAL_CAP) return -MOTOR_DUTY_FINAL_CAP;
    return duty;
}

/* 下板经 ENC_FEEDBACK 帧回传的编码器计数（调试/里程备用） */
volatile int32_t g_link_cnt_l = 0;
volatile int32_t g_link_cnt_r = 0;
/* 新速度样本就绪标志：OnEncFeedback 算出新窗口速度时置 1，速度环消费后清 0。
 * 让速度环按反馈实际刷新率(≈20Hz)闭环，而非主控制环 500Hz 重复积分陈旧值。 */
volatile uint8_t g_speed_sample_ready = 0;
static uint8_t g_prev_racing = 0;

/* 速度环控制器（定义于 PID_Controller.c），起步时给其初始输出做前馈 */
extern SpeedPID_Controller_t g_speed_pid;
extern PositionPID_Controller_t g_position_pid;
/* 位置环/速度环计算出的带符号电机目标（定义于 PID_Controller.c） */
extern volatile float g_motor_target_l;
extern volatile float g_motor_target_r;

/* 下板链路监视：收到 ENC_FEEDBACK 心跳清零；丢失超时则解除运行防窜车 */
static volatile uint8_t g_link_alive = 0;
static uint32_t g_link_lost_ticks = 0;
/* 心跳丢失阈值：下板 100Hz 回传，连续 50 个控制 tick(=100ms) 无心跳判为断链 */
#ifndef LINK_LOST_TICKS
#define LINK_LOST_TICKS 50u
#endif

/* 大端字节序解码辅助 */
#define PROTO_RD_U16(buf, i) ((int16_t)((uint16_t)(buf)[(i)] << 8 | (buf)[(i)+1]))

/* ==== 开环测试模式 ====
 * 置 1：按键直接给固定占空比、不跑 PID，用来验证"占空比→编码器读数"是否单调可信。
 *   K1=12%(1200)  K2=15%(1500)  K3=18%(1800)  K4=立即停。
 *   看 OLED 第3行 L/R 速度是否随占空比阶梯上升 → 编码器反馈可信，PID 才能闭环。
 * 置 0：恢复正常 K1 启动/K2 停/K3 复位的循迹模式。 */
#ifndef OPENLOOP_TEST_ENABLE
#define OPENLOOP_TEST_ENABLE 0
#endif
#if OPENLOOP_TEST_ENABLE
#define OPENLOOP_DUTY_12PCT  1200  /* 跨过左右电机起步死区 */
#define OPENLOOP_DUTY_15PCT  1500
#define OPENLOOP_DUTY_18PCT  1800
static int16_t g_openloop_duty   = 0;   /* 当前开环测试占空比(带符号，正=前进) */
static uint8_t g_openloop_active = 0;   /* 1=开环测试运行中 */
#endif

#ifndef OLED_TELEMETRY_ENABLE
#define OLED_TELEMETRY_ENABLE 1
#endif
#ifndef OLED_TELEMETRY_PERIOD_TICKS
#define OLED_TELEMETRY_PERIOD_TICKS 150u
#endif
/* 调试串口遥测：经 0x23 DEBUG_OUT 发到下板 USART3 → PC。
 * 与 OLED 同一周期（150 ticks ≈ 300ms），可按需关掉。 */
#ifndef DEBUG_OUT_TELEMETRY_ENABLE
#define DEBUG_OUT_TELEMETRY_ENABLE 1
#endif
#ifndef SENSOR_DEBUG_MIN_SPAN
#define SENSOR_DEBUG_MIN_SPAN 80u
#endif
#ifndef SENSOR_DEBUG_THRESHOLD_PERCENT
#define SENSOR_DEBUG_THRESHOLD_PERCENT 35u
#endif

static uint8_t g_sensor_min_index = 0;
static uint8_t g_sensor_max_index = 0;
static uint16_t g_sensor_min_value = 0;
static uint16_t g_sensor_max_value = 0;
static uint16_t g_sensor_span = 0;
static uint16_t g_sensor_low_mask = 0;
static uint16_t g_sensor_high_mask = 0;
static int16_t g_sensor_low_pos10 = -1;
static int16_t g_sensor_high_pos10 = -1;

static int16_t CalculateSensorDebugPos10(uint16_t mask, uint8_t low_is_target)
{
    uint8_t i;
    uint32_t weight_sum = 0;
    uint32_t position_sum = 0;
    for (i = 0; i < SENSOR_COUNT; i++)
    {
        uint32_t weight;
        if ((mask & (uint16_t)(1u << i)) == 0u)
        {
            continue;
        }
        if (low_is_target)
        {
            weight = (uint32_t)g_sensor_max_value - (uint32_t)g_line_sensor_values[i];
        }
        else
        {
            weight = (uint32_t)g_line_sensor_values[i] - (uint32_t)g_sensor_min_value;
        }
        weight++;
        weight_sum += weight;
        position_sum += weight * (uint32_t)i * 10u;
    }
    if (weight_sum == 0u)
    {
        return -1;
    }
    return (int16_t)((position_sum + weight_sum / 2u) / weight_sum);
}

static void UpdateSensorDebugSnapshot(void)
{
    uint8_t i;
    uint16_t low_threshold;
    uint16_t high_threshold;
    g_sensor_min_value = 0xFFFFu;
    g_sensor_max_value = 0u;
    g_sensor_low_mask = 0u;
    g_sensor_high_mask = 0u;
    g_sensor_low_pos10 = -1;
    g_sensor_high_pos10 = -1;
    for (i = 0; i < SENSOR_COUNT; i++)
    {
        uint16_t value = (uint16_t)g_line_sensor_values[i];
        if (value < g_sensor_min_value)
        {
            g_sensor_min_value = value;
            g_sensor_min_index = i;
        }
        if (value > g_sensor_max_value)
        {
            g_sensor_max_value = value;
            g_sensor_max_index = i;
        }
    }
    g_sensor_span = g_sensor_max_value - g_sensor_min_value;
    if (g_sensor_span < SENSOR_DEBUG_MIN_SPAN)
    {
        return;
    }
    low_threshold = g_sensor_min_value + (uint16_t)(((uint32_t)g_sensor_span * SENSOR_DEBUG_THRESHOLD_PERCENT) / 100u);
    high_threshold = g_sensor_max_value - (uint16_t)(((uint32_t)g_sensor_span * SENSOR_DEBUG_THRESHOLD_PERCENT) / 100u);
    for (i = 0; i < SENSOR_COUNT; i++)
    {
        uint16_t value = (uint16_t)g_line_sensor_values[i];
        if (value <= low_threshold)
        {
            g_sensor_low_mask |= (uint16_t)(1u << i);
        }
        if (value >= high_threshold)
        {
            g_sensor_high_mask |= (uint16_t)(1u << i);
        }
    }
    g_sensor_low_pos10 = CalculateSensorDebugPos10(g_sensor_low_mask, 1u);
    g_sensor_high_pos10 = CalculateSensorDebugPos10(g_sensor_high_mask, 0u);
}

static void StopRun(void)
{
    is_racing = 0;
    lose_time = 0;
}

#if OPENLOOP_TEST_ENABLE
static void OpenLoop_Set(uint8_t active, int16_t duty)
{
    g_openloop_active = active;
    g_openloop_duty = active ? duty : 0;
    g_motor_target_l = (float)g_openloop_duty;
    g_motor_target_r = (float)g_openloop_duty;
    RGB_SetColor(active ? RGB_COLOR_G : RGB_COLOR_R);
}

static void OpenLoop_ShowStatus(void)
{
    int spd_l = (int)speed_left;
    int spd_r = (int)speed_right;
    if (spd_l > 999) spd_l = 999;
    if (spd_l < -999) spd_l = -999;
    if (spd_r > 999) spd_r = 999;
    if (spd_r < -999) spd_r = -999;

    OLED_ClearLine(1);
    OLED_ShowString(1, 1, g_openloop_active ? "OPENLOOP RUN    " : "OPENLOOP STOP   ");

    OLED_ClearLine(2);
    OLED_ShowString(2, 1, "K:");
    OLED_ShowChar(2, 3, Key_GetState(KEY_K1) ? '1' : '0');
    OLED_ShowChar(2, 4, Key_GetState(KEY_K2) ? '2' : '0');
    OLED_ShowChar(2, 5, Key_GetState(KEY_K3) ? '3' : '0');
    OLED_ShowChar(2, 6, Key_GetState(KEY_K4) ? '4' : '0');
    OLED_ShowString(2, 8, "D:");
    OLED_ShowSignedNum(2, 10, g_openloop_duty, 4);

    OLED_ClearLine(3);
    OLED_ShowString(3, 1, "L:");
    OLED_ShowSignedNum(3, 3, spd_l, 3);
    OLED_ShowString(3, 8, "R:");
    OLED_ShowSignedNum(3, 10, spd_r, 3);

    OLED_ClearLine(4);
    OLED_ShowString(4, 1, g_link_alive ? "LINK:OK " : "LINK:-- ");
    OLED_ShowString(4, 9, g_openloop_active ? "A:1" : "A:0");
}

static void OpenLoop_HandleKeys(void)
{
    uint8_t next_active = g_openloop_active;
    int16_t next_duty = g_openloop_duty;

    if (Key_GetState(KEY_K4))
    {
        next_active = 0;
        next_duty = 0;
    }
    else if (Key_GetState(KEY_K3))
    {
        next_active = 1;
        next_duty = OPENLOOP_DUTY_18PCT;
    }
    else if (Key_GetState(KEY_K2))
    {
        next_active = 1;
        next_duty = OPENLOOP_DUTY_15PCT;
    }
    else if (Key_GetState(KEY_K1))
    {
        next_active = 1;
        next_duty = OPENLOOP_DUTY_12PCT;
    }

    if (next_active != g_openloop_active || next_duty != g_openloop_duty)
    {
        OpenLoop_Set(next_active, next_duty);
        OpenLoop_ShowStatus();
    }
}
#endif

// ========== ESP32 协议回调（保留兼容） ==========
static void OnLoraStop(const ProtoFrame_t *f)
{
    (void)f;
    StopRun();
}

static void OnRadarDist(const ProtoFrame_t *f)
{
    if (f->len < 2) return;
    radar_distance_cm = (uint16_t)PROTO_RD_U16(f->payload, 0);
}

// 下板 -> 上板 编码器反馈帧：更新轮速（供本地速度环使用）
// 速度窗口：下板 100Hz(10ms)回传，单帧增量太小(≈0.6)且量化严重。
// 用 SPEED_WIN_MS 的滑动窗口累计计数，再按实测耗时换算成"计数/秒"，
// 量化噪声降到单帧的 1/(窗口帧数)，同时仍是物理上可解释的速度量纲。
#ifndef SPEED_WIN_MS
#define SPEED_WIN_MS 50u
#endif
static void OnEncFeedback(const ProtoFrame_t *f)
{
    int32_t cnt_l, cnt_r, dl, dr;
    static uint8_t cnt_inited = 0;
    static int32_t last_cnt_l = 0, last_cnt_r = 0;
    static int32_t vel_accum_l = 0, vel_accum_r = 0;
    static uint32_t vel_t0 = 0;

    if (f->len < PROTO_ENC_FEEDBACK_LEN) return;

    cnt_l = (int32_t)(((uint32_t)f->payload[4] << 24) | ((uint32_t)f->payload[5] << 16) |
                      ((uint32_t)f->payload[6] << 8)  |  (uint32_t)f->payload[7]);
    cnt_r = (int32_t)(((uint32_t)f->payload[8] << 24) | ((uint32_t)f->payload[9] << 16) |
                      ((uint32_t)f->payload[10] << 8) |  (uint32_t)f->payload[11]);
    g_link_cnt_l = cnt_l;
    g_link_cnt_r = cnt_r;

    if (!cnt_inited)
    {
        cnt_inited = 1;
        last_cnt_l = cnt_l;
        last_cnt_r = cnt_r;
        vel_accum_l = 0;
        vel_accum_r = 0;
        vel_t0 = Millis_Get();
        speed_left = 0;
        speed_right = 0;
    }
    else
    {
        /* 本帧原始增量：里程按它累积(每帧恰好计一次，单位=计数) */
        dl = cnt_l - last_cnt_l;
        dr = cnt_r - last_cnt_r;
        last_cnt_l = cnt_l;
        last_cnt_r = cnt_r;
        Path_UpdateOdometer(dl, dr);

        /* 速度窗口累计，到窗口期再换算成"计数/秒"，降低低速量化噪声 */
        vel_accum_l += dl;
        vel_accum_r += dr;
        {
            uint32_t now = Millis_Get();
            uint32_t dt  = now - vel_t0;          /* 实测耗时(ms)，毫秒回绕安全 */
            if (dt >= SPEED_WIN_MS)
            {
                speed_left  = (int16_t)((vel_accum_l * 1000) / (int32_t)dt);
                speed_right = (int16_t)((vel_accum_r * 1000) / (int32_t)dt);
                vel_accum_l = 0;
                vel_accum_r = 0;
                vel_t0 = now;
                g_speed_sample_ready = 1;     /* 通知速度环：有新样本可闭环 */
            }
        }
    }

    g_link_alive = 1;          /* 收到下板心跳 */
    g_link_lost_ticks = 0;
}

// 下板 -> 上板 链路复位：下板刚开机/重烧，必须解除运行防止用陈旧状态窜车
static void OnLinkReset(const ProtoFrame_t *f)
{
    (void)f;
    StopRun();                 /* is_racing=0，停止下发使能 */
    SpeedPID_Reset(&g_speed_pid);
    PositionPID_Reset(&g_position_pid);
    g_link_alive = 1;
    g_link_lost_ticks = 0;
    RGB_SetColor(RGB_COLOR_R);
    OLED_ShowString(1, 1, "LOWER RESET     ");
}

int main(void)
{
    // 外设初始化（传感/显示/交互 + 本地控制算法，电机PWM输出经串口转发）
    RGB_Init();
    OLED_Init();
    g_imu_init_ok = MPU6050_Init();
    g_imu_who_id  = MPU6050_ReadID();
    SysTick_Init();
    BlackPoint_Finder_Init();
    LineSensor_Init();
    Key_Scan_Init();
    Uart2_Init(115200);
    TelemetryScreen_Init();
    PID_Init();
    Path_Init();

    (void)g_imu_init_ok;
    (void)g_imu_who_id;
    (void)g_sensor_low_pos10;
    (void)g_sensor_high_pos10;

    // 板间/ESP32 协议初始化
    Proto_Init();
    Proto_RegisterHandler(PROTO_CMD_LORA_STOP, OnLoraStop);
    Proto_RegisterHandler(PROTO_CMD_RADAR_DIST, OnRadarDist);
    Proto_RegisterHandler(PROTO_CMD_ENC_FEEDBACK, OnEncFeedback);
    Proto_RegisterHandler(PROTO_CMD_LINK_RESET, OnLinkReset);

    /* 上板开机/重烧：通告下板复位到安全态（停车下电、清里程），
     * 避免上板重启瞬间下板仍按陈旧指令运行。多发几帧防丢包。 */
    is_racing = 0;
    Proto_SendLinkReset();
    Delay_ms(5);
    Proto_SendLinkReset();

#if OLED_TELEMETRY_ENABLE
    uint8_t oled_due = 1;
    uint32_t oled_last_tick = add_angle_num;
#endif

    while (1)
    {
        // ---- 主循环控制环：由 SysTick 标志位驱动（2ms） ----
        if (g_control_tick)
        {
            g_control_tick = 0;

            // 7路灰度 + PA0电池电压轮询采样
            LineSensor_SampleAll();
            UpdateSensorDebugSnapshot();

            // 黑线识别 -> 循迹位置
            BlackPoint_Finder_Search(g_line_sensor_values, &result_BlackPoint);
            if (result_BlackPoint.found)
            {
                position_get = (int16_t)(result_BlackPoint.precise_position * 10.0f);
                if (lose_time > 0) lose_time--;
            }
            else
            {
                lose_time++;
                if (lose_time > 500)
                {
                    lose_time = 500;
                    StopRun();             /* 丢线超时：本地停车 */
                }
            }

            // racing 上升沿：复位 Path/PID 并给速度环 15% 前馈起步
            if (is_racing && !g_prev_racing)
            {
                Path_StartRace();
                lose_time = 0;
                g_speed_pid.last_output = START_DUTY_15PCT;
            }
            else if (!is_racing && g_prev_racing)
            {
                Path_StopRace();
            }
            g_prev_racing = is_racing;

            // 下板心跳监视：运行中若下板断链(掉电/重烧/线松)，解除运行防窜车
            if (g_link_alive)
            {
                g_link_lost_ticks++;
                if (g_link_lost_ticks > LINK_LOST_TICKS)
                {
                    g_link_lost_ticks = LINK_LOST_TICKS + 1u;
                    if (is_racing)
                    {
                        StopRun();
                        SpeedPID_Reset(&g_speed_pid);
                        PositionPID_Reset(&g_position_pid);
                        RGB_SetColor(RGB_COLOR_R);
                        OLED_ShowString(1, 1, "LINK LOST STOP  ");
                    }
                    g_link_alive = 0;   /* 等待下板心跳/LINK_RESET 重新置位 */
                }
            }

            // 双环 PID 控制（is_racing=0 时内部自动停车并复位 g_motor_target=0）
            PID_Control_Update();

#if OPENLOOP_TEST_ENABLE
            // 开环测试：跳过 PID 输出，直接下发固定占空比，观察编码器原始响应。
            // 开环 duty 是绝对占空比，不走 PID 调节量上限(1000)，否则 1200/1500/1800 会被压成同一个值。
            g_motor_target_l = (float)g_openloop_duty;
            g_motor_target_r = (float)g_openloop_duty;
            {
                int16_t duty = (int16_t)ClampMotorDutyFinal((float)g_openloop_duty);
                Proto_SendMotorCmd(duty, duty, g_openloop_active);
            }
#else
            // 下发电机指令到下板执行器（带符号占空比 + 使能）
            // 顺序：先对 PID 输出限幅(约束调节量) -> 加左右死区前馈(抬到电机能动区间)
            //      -> 总量再限到下板安全上限。这样 PID 的有效调节范围完整保留，死区只是平移。
            {
                float duty_l = ApplyDeadzone(ClampClosedLoopDuty(g_motor_target_l), MOTOR_DEADZONE_L);
                float duty_r = ApplyDeadzone(ClampClosedLoopDuty(g_motor_target_r), MOTOR_DEADZONE_R);
                Proto_SendMotorCmd((int16_t)ClampMotorDutyFinal(duty_l),
                                   (int16_t)ClampMotorDutyFinal(duty_r),
                                   is_racing);
            }
#endif
        }

        {
            uint32_t now_tick = add_angle_num;
#if OLED_TELEMETRY_ENABLE
            if ((uint32_t)(now_tick - oled_last_tick) >= OLED_TELEMETRY_PERIOD_TICKS)
            {
                oled_last_tick = now_tick;
                oled_due = 1;
            }
#else
            (void)now_tick;
#endif
        }

        // ---- 板间协议接收处理（编码器反馈 + ESP32/雷达兼容） ----
        Proto_Process();

        // ---- 人机交互：按键启动/停止 ----
        Key_Scan_Update();
        Key_Event_t *event = Key_GetEvent();
        BDI_V = (float)g_battery_adc_value * 0.00426508726f;

#if OPENLOOP_TEST_ENABLE
        OpenLoop_HandleKeys();
#endif

        if (event != NULL)
        {
#if OPENLOOP_TEST_ENABLE
            (void)event;
#else
            switch (event->key_id)
            {
            case KEY_NONE:
                break;
            case KEY_K1:
                // K1: 启动运行
                is_racing = 1;
                lose_time = 0;
                BlackPoint_Finder_ResetLastPosition();
                RGB_SetColor(RGB_COLOR_G);
                OLED_ShowString(1, 1, "RUN  K1 START   ");
                break;
            case KEY_K2:
                // K2: 停止运行
                StopRun();
                RGB_SetColor(RGB_COLOR_R);
                OLED_ShowString(1, 1, "STOP K2         ");
                break;
            case KEY_K3:
                // K3: 复位丢线/位置（调试用，不启动）
                StopRun();
                BlackPoint_Finder_ResetLastPosition();
                OLED_ShowString(1, 1, "KEY=K3 RESET    ");
                break;
            case KEY_K4:
                OLED_ShowString(1, 1, "KEY=K4          ");
                break;
            }
#endif
        }

#if OLED_TELEMETRY_ENABLE
        if (oled_due)
        {
            oled_due = 0;
#if OPENLOOP_TEST_ENABLE
            OpenLoop_ShowStatus();
#else
            TelemetryScreen_Update();
#endif
#if DEBUG_OUT_TELEMETRY_ENABLE
            {
                char dbg[64];
                int n = snprintf(dbg, sizeof(dbg),
                    "L=%d R=%d T=%d out=%d dl=%d dr=%d\r\n",
                    (int)speed_left, (int)speed_right,
                    (int)PID_GetCurrentTargetSpeed(),
                    (int)g_speed_pid.last_output,
                    (int)g_motor_target_l, (int)g_motor_target_r);
                if (n > 0 && n < (int)sizeof(dbg))
                    Proto_SendDebugOut((const uint8_t *)dbg, (uint8_t)n);
            }
#endif
        }
#endif
    }
}
