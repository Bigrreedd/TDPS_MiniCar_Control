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
#include "stm32f10x_it.h"

// 滴答定时器初始化，2ms中断一次 (500Hz)
void SysTick_Init(void)
{
    SysTick_Config(SystemCoreClock / 500);
}

float BDI_V = 0;
uint8_t is_racing = 0;

// 控制环状态（从 SysTick 移到主循环）
extern int16_t position_get;
BlackPointResult_t result_BlackPoint;
static uint32_t lose_time = 0;

// 手动驾驶模式（K3/K4 使用）
static void StartManualDrive(uint16_t motor_duty)
{
    is_racing = 0;
    Path_StopRace();
    g_manual_drive_active = 1;
    g_manual_drive_ticks_remaining = 1000; // 2 seconds
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
    }
}