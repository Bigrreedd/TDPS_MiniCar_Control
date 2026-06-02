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

/* 起步占空比：满量程 10000 的 15% */
#define START_DUTY_15PCT  1500.0f

/* 下板经 ENC_FEEDBACK 帧回传的编码器计数（调试/里程备用） */
volatile int32_t g_link_cnt_l = 0;
volatile int32_t g_link_cnt_r = 0;
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

#ifndef OLED_TELEMETRY_ENABLE
#define OLED_TELEMETRY_ENABLE 1
#endif
#ifndef OLED_TELEMETRY_PERIOD_TICKS
#define OLED_TELEMETRY_PERIOD_TICKS 150u
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
static void OnEncFeedback(const ProtoFrame_t *f)
{
    if (f->len < PROTO_ENC_FEEDBACK_LEN) return;
    speed_left  = PROTO_RD_U16(f->payload, 0);
    speed_right = PROTO_RD_U16(f->payload, 2);
    g_link_cnt_l = (int32_t)(((uint32_t)f->payload[4] << 24) | ((uint32_t)f->payload[5] << 16) |
                             ((uint32_t)f->payload[6] << 8)  |  (uint32_t)f->payload[7]);
    g_link_cnt_r = (int32_t)(((uint32_t)f->payload[8] << 24) | ((uint32_t)f->payload[9] << 16) |
                             ((uint32_t)f->payload[10] << 8) |  (uint32_t)f->payload[11]);
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

            // 下发电机指令到下板执行器（带符号占空比 + 使能）
            Proto_SendMotorCmd((int16_t)g_motor_target_l,
                               (int16_t)g_motor_target_r,
                               is_racing);
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

        if (event != NULL)
        {
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
        }

#if OLED_TELEMETRY_ENABLE
        if (oled_due)
        {
            oled_due = 0;
            TelemetryScreen_Update();
        }
#endif
    }
}
