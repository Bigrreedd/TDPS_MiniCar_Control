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
#include "stm32f10x_it.h"

/*
 * ============================================================
 *  二合一板 —— 上板（传感/“手”）固件
 *  职责：7 路灰度循迹 + MPU6050 + OLED + 按键启动，
 *        通过 USART2 把 [循迹位置 / 找线标志 / 启停 / 角速度]
 *        发送给下板（电机/“脚”）。
 *  本板不接电机、编码器、风扇，相关逻辑全部交给下板。
 *  注意：USART2 现在是“板间链路”，只发二进制 SENSOR_DATA 帧，
 *        不再输出人类可读调试文本，避免污染下板解析器。
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

/* 下板经 MOTOR_STATUS 帧回传的电机数据，供 OLED 显示真实轮速/占空比 */
volatile int16_t g_link_spd_l = 0;
volatile int16_t g_link_spd_r = 0;
volatile uint8_t g_link_pwm_pct = 0;
volatile uint8_t g_link_seg = 0;

/* 大端字节序解码辅助 */
#define PROTO_RD_U16(buf, i) ((int16_t)((uint16_t)(buf)[(i)] << 8 | (buf)[(i)+1]))

/* SENSOR_DATA 发送周期：每 N 个 2ms 控制 tick 发一帧。
 * 2 tick = 4ms ≈ 250Hz，兼顾循迹实时性与链路占用（约 22%@115200）。 */
#ifndef SENSOR_DATA_PERIOD_TICKS
#define SENSOR_DATA_PERIOD_TICKS 2u
#endif
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

// ========== ESP32 协议回调（保留兼容，本板不驱动电机） ==========
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

// 下板 -> 上板 电机状态帧
static void OnMotorStatus(const ProtoFrame_t *f)
{
    if (f->len < PROTO_MOTOR_STATUS_LEN) return;
    g_link_spd_l   = PROTO_RD_U16(f->payload, 0);
    g_link_spd_r   = PROTO_RD_U16(f->payload, 2);
    g_link_pwm_pct = f->payload[4];
    g_link_seg     = f->payload[5];
}

int main(void)
{
    // 外设初始化（仅传感/显示/交互，无电机/编码器）
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

    (void)g_imu_init_ok;
    (void)g_imu_who_id;
    (void)g_sensor_low_pos10;
    (void)g_sensor_high_pos10;

    // 板间/ESP32 协议初始化
    Proto_Init();
    Proto_RegisterHandler(PROTO_CMD_LORA_STOP, OnLoraStop);
    Proto_RegisterHandler(PROTO_CMD_RADAR_DIST, OnRadarDist);
    Proto_RegisterHandler(PROTO_CMD_MOTOR_STATUS, OnMotorStatus);

    uint8_t sensor_due = 0;
    uint32_t sensor_last_tick = add_angle_num;
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
                    lose_time = 500;       /* 丢线由下板根据 found 标志决定是否停车 */
                }
            }
        }

        {
            uint32_t now_tick = add_angle_num;
            if ((uint32_t)(now_tick - sensor_last_tick) >= SENSOR_DATA_PERIOD_TICKS)
            {
                sensor_last_tick = now_tick;
                sensor_due = 1;
            }
#if OLED_TELEMETRY_ENABLE
            if ((uint32_t)(now_tick - oled_last_tick) >= OLED_TELEMETRY_PERIOD_TICKS)
            {
                oled_last_tick = now_tick;
                oled_due = 1;
            }
#endif
        }

        // ---- 板间协议接收处理（ESP32/雷达兼容，低优先级） ----
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
                // K1: 启动运行（通知下板开始循迹/PID）
                is_racing = 1;
                lose_time = 0;
                BlackPoint_Finder_ResetLastPosition();
                RGB_SetColor(RGB_COLOR_G);
                OLED_ShowString(1, 1, "RUN  K1 START   ");
                break;
            case KEY_K2:
                // K2: 停止运行（通知下板停车）
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

        // ---- 发送传感数据帧到下板（二进制，板间链路专用） ----
        if (sensor_due)
        {
            sensor_due = 0;
            Proto_SendSensorData(position_get,
                                 result_BlackPoint.found,
                                 is_racing,
                                 MPU6050_data.gz_rads);
        }
    }
}
