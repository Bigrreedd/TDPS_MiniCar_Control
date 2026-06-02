#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "Motor_ctr.h"
#include "ABEncoder.h"
#include "Uart_Config.h"
#include "stdio.h"
#include "math.h"
#include "system_stm32f10x.h"
#include "BlackPoint_Finder.h"
#include "PID_Controller.h"
#include "Path.h"
#include "Protocol.h"
#include "stm32f10x_it.h"

/*
 * ============================================================
 *  二合一板 —— 下板（电机/“脚”）固件
 *  职责：通过 USART2 接收上板发来的 SENSOR_DATA（循迹位置 /
 *        找线标志 / 启停 / 角速度），结合本地编码器测速，
 *        运行 legacy 的位置环 + 速度环双环 PID 驱动两路电机
 *        (DRV8701: PA8/9/10/11 方向+PWM, PA12 使能)。
 *  本板不接灰度/OLED/IMU/按键/风扇，数据全部来自上板串口。
 *  起步占空比约 15%（在 racing 上升沿给速度环一个初始输出）。
 * ============================================================
 */

// 滴答定时器初始化，2ms中断一次 (500Hz)
void SysTick_Init(void)
{
    SysTick_Config(SystemCoreClock / 500);
}

volatile uint8_t is_racing = 0;
extern volatile uint16_t uart_rx_timeout;   /* 定义于 Motor_ctr.c，串口看门狗计数 */

// 控制环状态
BlackPointResult_t result_BlackPoint;     /* found 由串口帧填充，供 Path 使用 */
static uint32_t lose_time = 0;
static uint8_t  g_prev_racing = 0;

/* 起步占空比：满量程 10000 的 15% */
#define START_DUTY_15PCT  1500.0f

/* 回传电机状态周期：每 N 个 2ms tick 发一帧（5 tick = 10ms = 100Hz，OLED 显示足够） */
#ifndef MOTOR_STATUS_PERIOD_TICKS
#define MOTOR_STATUS_PERIOD_TICKS 5u
#endif

/* 速度环控制器（定义在 PID_Controller.c），起步时给其初始输出做前馈 */
extern SpeedPID_Controller_t g_speed_pid;

/* 大端字节序解码辅助 */
#define PROTO_RD_U16(buf, i) ((int16_t)((uint16_t)(buf)[(i)] << 8 | (buf)[(i)+1]))

// ESP32-S3 雷达数据（保留协议兼容）
volatile uint16_t radar_distance_cm = 0;

static void StopAll(void)
{
    Path_StopRace();
    Motor_StopAll();
    Motor_Disable();
    is_racing = 0;
    g_prev_racing = 0;
    lose_time = 0;
}

// ========== 上板 -> 下板 传感数据帧 ==========
static void OnSensorData(const ProtoFrame_t *f)
{
    if (f->len < PROTO_SENSOR_DATA_LEN) return;

    int16_t pos   = PROTO_RD_U16(f->payload, 0);
    uint8_t flags = f->payload[2];
    int16_t gz_scaled = PROTO_RD_U16(f->payload, 4);

    /* 喂给控制环的输入 */
    position_get = pos;
    result_BlackPoint.found = (flags & PROTO_SENSOR_FLAG_FOUND) ? 1u : 0u;
    g_link_gz_rads = (float)gz_scaled / 1000.0f;

    uint8_t want_racing = (flags & PROTO_SENSOR_FLAG_RACING) ? 1u : 0u;

    /* 喂狗：链路有数据 */
    uart_rx_timeout = 0;

    if (want_racing)
    {
        is_racing = 1;
        lose_time = 0;
    }
    else
    {
        if (is_racing) StopAll();
    }
}

// ========== ESP32 协议回调（保留兼容） ==========
static void OnLoraStop(const ProtoFrame_t *f)
{
    (void)f;
    StopAll();
}

static void OnRadarDist(const ProtoFrame_t *f)
{
    if (f->len < 2) return;
    radar_distance_cm = (uint16_t)PROTO_RD_U16(f->payload, 0);
}

int main(void)
{
    // 外设初始化（仅电机/编码器/串口）
    SysTick_Init();
    Motor_Init();
    ABEncoder_Init();
    BlackPoint_Finder_Init();
    Uart2_Init(115200);
    PID_Init();
    Path_Init();

    // 板间/ESP32 协议初始化
    Proto_Init();
    Proto_RegisterHandler(PROTO_CMD_SENSOR_DATA, OnSensorData);
    Proto_RegisterHandler(PROTO_CMD_LORA_STOP, OnLoraStop);
    Proto_RegisterHandler(PROTO_CMD_RADAR_DIST, OnRadarDist);

    Motor_StopAll();
    Motor_Disable();

    uint32_t status_last_tick = add_angle_num;

    while (1)
    {
        // ---- 主循环控制环：由 SysTick 标志位驱动（2ms） ----
        if (g_control_tick)
        {
            g_control_tick = 0;

            // racing 上升沿：使能电机 + 给速度环 15% 前馈起步
            if (is_racing && !g_prev_racing)
            {
                Motor_Enable();
                Path_StartRace();
                lose_time = 0;
                /* 速度环增量式输出从此初值起步，立即给约 15% 占空 */
                g_speed_pid.last_output = START_DUTY_15PCT;
            }
            g_prev_racing = is_racing;

            if (is_racing)
            {
                // 丢线保护：上板长时间报告未找到线则停车
                if (result_BlackPoint.found)
                {
                    if (lose_time > 0) lose_time--;
                }
                else
                {
                    lose_time++;
                    if (lose_time > 500)
                    {
                        lose_time = 500;
                        StopAll();
                    }
                }
            }

            // 双环 PID 控制（is_racing=0 时内部自动停车并复位）
            PID_Control_Update();
        }

        // ---- 周期回传电机状态给上板（全双工链路） ----
        {
            uint32_t now_tick = add_angle_num;
            if ((uint32_t)(now_tick - status_last_tick) >= MOTOR_STATUS_PERIOD_TICKS)
            {
                uint32_t duty;
                uint8_t pwm_pct;
                status_last_tick = now_tick;
                duty = ((uint32_t)Motor_GetDuty(MOTOR_L) + (uint32_t)Motor_GetDuty(MOTOR_R)) / 2u;
                pwm_pct = (uint8_t)((duty * 100u) / MOTOR_DUTY_MAX);
                if (pwm_pct > 100u) pwm_pct = 100u;
                Proto_SendMotorStatus(speed_left, speed_right, pwm_pct,
                                      (uint8_t)Path_GetCurrentSegment(), is_racing);
            }
        }

        // ---- 板间协议接收处理 ----
        Proto_Process();
    }
}
