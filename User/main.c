#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "M3PWM.h"
#include "Motor_ctr.h"
#include "OLED.h"
#include "ABEncoder.h"
#include "ADC_get.h"
#include "Uart_Config.h"
#include "stdio.h"
#include "math.h"
#include "RGB_Led.h"
#include "LSM6DSR_Config.h"
#include "pose.h"
#include "Key_Scan.h"
#include "system_stm32f10x.h"
#include "BlackPoint_Finder.h"
#include "PID_Controller.h"
#include "Path.h"
#include "TelemetryScreen.h"
#include "Protocol.h"
#include "Battery.h"
#include "stm32f10x_it.h"

// 滴答定时器初始化，2ms中断一次 (500Hz)
void SysTick_Init(void)
{
    SysTick_Config(SystemCoreClock / 500);
}

float BDI_V = 0;
volatile uint8_t is_racing = 0;

// 控制环状态（从 SysTick 移到主循环）
// position_get 的 extern 声明已在 stm32f10x_it.h 中
BlackPointResult_t result_BlackPoint;
static uint32_t lose_time = 0;

// ESP32-S3 雷达数据
volatile uint16_t radar_distance_cm = 0;

/* 大端字节序解码辅助 */
#define PROTO_RD_U16(buf, i) ((int16_t)((uint16_t)(buf)[(i)] << 8 | (buf)[(i)+1]))

// ========== ESP32 协议回调 ==========
static void OnLoraSpeed(const ProtoFrame_t *f)
{
    if (f->len < 4) return;
    int16_t left  = PROTO_RD_U16(f->payload, 0);
    int16_t right = PROTO_RD_U16(f->payload, 2);
    /* 遥控模式：退出自动循迹，直接驱动电机 */
    is_racing = 0;
    Path_StopRace();
    Motor_Enable();
    Motor_SetSpeedWithDirection(MOTOR_L, (float)left);
    Motor_SetSpeedWithDirection(MOTOR_R, (float)right);
}

static void OnLoraStop(const ProtoFrame_t *f)
{
    (void)f;
    Path_StopRace();
    Motor_StopAll();
    Motor_Disable();
    is_racing = 0;
}

static void OnRadarDist(const ProtoFrame_t *f)
{
    if (f->len < 2) return;
    radar_distance_cm = (uint16_t)PROTO_RD_U16(f->payload, 0);
}

// 手动驾驶模式（K3/K4 使用）
static void StartManualDrive(uint16_t motor_duty)
{
    is_racing = 0;
    Path_StopRace();
    g_manual_drive_ticks_remaining = 1000; // 2 seconds — set BEFORE active to avoid ISR race
    g_manual_drive_active = 1;
    Motor_Enable();
    Motor_SetDirection(MOTOR_L, MOTOR_DIR_FORWARD);
    Motor_SetDirection(MOTOR_R, MOTOR_DIR_FORWARD);
    Motor_SetSpeed(MOTOR_L, motor_duty);
    Motor_SetSpeed(MOTOR_R, motor_duty);
}

int main(void)
{
    // 外设初始化
    RGB_Init();
    OLED_Init();
    LSM6DSR_Init();
    M3PWM_Init();
    SysTick_Init();
    Motor_Init();
    M3PWM_Start();
    ABEncoder_Init();
    BlackPoint_Finder_Init();
    MuxADC_Init();
    Key_Scan_Init();
    Uart2_Init(115200);
    PID_Init();
    Path_Init();
    TelemetryScreen_Init();

    // ESP32-S3 协议初始化
    Proto_Init();
    Proto_RegisterHandler(PROTO_CMD_LORA_SPEED, OnLoraSpeed);
    Proto_RegisterHandler(PROTO_CMD_LORA_STOP, OnLoraStop);
    Proto_RegisterHandler(PROTO_CMD_RADAR_DIST, OnRadarDist);

    while (1)
    {
        // ---- 主循环控制环：由 SysTick 标志位驱动 ----
        if (g_control_tick)
        {
            g_control_tick = 0;

            // ADC 16+1 通道轮询采样（耗时操作，已从中断移出）
            MuxADC_SampleAll();

            if (!g_manual_drive_active)
            {
                // 黑线识别
                BlackPoint_Finder_Search(g_mux_adc_values, &result_BlackPoint);

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
                        Motor_Disable();
                        is_racing = 0;
                    }
                }

                // 双环 PID 控制
                PID_Control_Update();
            }
        }

        // ---- ESP32-S3 协议处理（非实时，低优先级） ----
        Proto_Process();

        // ---- 人机交互（非实时，低优先级） ----
        Key_Scan_Update();
        Key_Event_t *event = Key_GetEvent();
        BDI_V = (float)g_mux_adc_values[16] * 0.00426508726f;

        if (event != NULL)
        {
            switch (event->key_id)
            {
            case KEY_NONE:
                break;
            case KEY_K1:
                // K1: 启动自动循迹模式
                RGB_SetColor(RGB_COLOR_R);
                Motor_Enable();
                is_racing = 1;
                lose_time = 0;  // 复位丢线超时计数器，避免超时后重启立即再超时
                Path_StartRace();
                break;
            case KEY_K2:
                // K2: 立即停止所有运动
                RGB_SetColor(RGB_COLOR_G);
                Path_StopRace();
                Motor_StopAll();
                Motor_Disable();
                lose_time = 0;
                break;
            case KEY_K3:
                // K3: 手动前进 20%
                RGB_SetColor(RGB_COLOR_YELLOW);
                StartManualDrive((uint16_t)(MOTOR_DUTY_MAX * 20u / 100u));
                break;
            case KEY_K4:
                // K4: 手动前进 40%
                RGB_SetColor(RGB_COLOR_CYAN);
                StartManualDrive((uint16_t)(MOTOR_DUTY_MAX * 40u / 100u));
                break;
            }
        }

        TelemetryScreen_Update();

        // 每 ~200ms 发送一次遥测到 ESP32（100 ticks * 2ms）
        {
            static uint16_t telem_cnt = 0;
            if (++telem_cnt >= 100) {
                telem_cnt = 0;
                Proto_SendTelemetry(position_get, speed_left, speed_right,
                                    (uint8_t)Path_GetCurrentSegment(), battery_percent(BDI_V));
            }
        }
    }
}