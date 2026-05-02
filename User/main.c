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
uint8_t star_car = 0;

// 手动驾驶模式（K3/K4 使用）
static void StartManualDrive(uint16_t motor_duty)
{
    star_car = 0;
    Path_StopRace();               // 确保自动循迹已停止
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
                RGB_SetColor(1);
                Motor_Enable();
                star_car = 1;
                Path_StartRace();
                break;
            case KEY_K2:
                // K2: 立即停止所有运动
                RGB_SetColor(2);
                Path_StopRace();
                Motor_StopAll();
                Motor_Disable();
                break;
            case KEY_K3:
                // K3: 手动前进 20%
                RGB_SetColor(4);
                StartManualDrive((uint16_t)(MOTOR_DUTY_MAX * 20u / 100u));
                break;
            case KEY_K4:
                // K4: 手动前进 40%
                RGB_SetColor(5);
                StartManualDrive((uint16_t)(MOTOR_DUTY_MAX * 40u / 100u));
                break;
            }
        }

        TelemetryScreen_Update();
    }
}