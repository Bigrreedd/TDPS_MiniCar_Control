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
#include "TelemetryScreen.h"
#include "stm32f10x_it.h"

// 滴答定时器初始化，1ms中断一次
void SysTick_Init(void)
{
    // 配置SysTick为1ms中断（SystemCoreClock / 1000）
    SysTick_Config(SystemCoreClock / 500);
}

float BDI_V = 0;
uint8_t star_car = 0;

static void StartManualDrive(uint16_t motor_duty)
{
    star_car = 0;
    g_manual_drive_active = 1;
    g_manual_drive_ticks_remaining = 1000; // 2 seconds
    M3PWM_SetDutyCycle(950);
    Motor_Enable();
    Motor_SetDirection(MOTOR_L, MOTOR_DIR_FORWARD);
    Motor_SetDirection(MOTOR_R, MOTOR_DIR_FORWARD);
    Motor_SetSpeed(MOTOR_L, motor_duty);
    Motor_SetSpeed(MOTOR_R, motor_duty);
}

int main(void)
{
    //RGB初始化
    RGB_Init();
    //屏幕初始化
    OLED_Init();
    LSM6DSR_Init();
    // 初始化PWM模块
    M3PWM_Init();
    //滴答定时器初始化
    SysTick_Init();
    // 初始化电机控制模块
    Motor_Init();
    // 启动PWM输出
    M3PWM_Start();
    //初始化编码器
    ABEncoder_Init();
    // 使能电机
    BlackPoint_Finder_Init();
    //光电管初始化
    MuxADC_Init();
    Key_Scan_Init();
    //串口2初始化
    Uart2_Init(115200);
    PID_Init();
    TelemetryScreen_Init();
    //Delay_s(5);
    /*主循环，循环体内的代码会一直循环执行*/
    while (1)
	{
		Key_Scan_Update();
		Key_Event_t *event = Key_GetEvent();
		BDI_V = (float)g_mux_adc_values[16] * 0.00426508726f;
		if(event != NULL)
		{
			switch(event->key_id)
			{
				case KEY_NONE:
					break;
				case KEY_K1:
					RGB_SetColor(1);
					StartManualDrive((uint16_t)(MOTOR_DUTY_MAX * 20u / 100u));
					break;
				case KEY_K2:
					RGB_SetColor(2);
					StartManualDrive((uint16_t)(MOTOR_DUTY_MAX * 40u / 100u));
					break;
				case KEY_K3:
					RGB_SetColor(4);
					break;
				case KEY_K4:
					RGB_SetColor(5);
					break;
			}
		}

		TelemetryScreen_Update();
//		else
//		{
//			while(1);
//		}
		//Delay_s(10);
//		OLED_ShowSignedNum(1,1,speed_left,10);
		//OLED_ShowSignedNum(1,1,2222222,10);
//		OLED_Clear();
//		Delay_ms(2000);
//		/*PWM占空比调节演示*/
//		// 设置PWM占空比为25%
//		M3PWM_SetDutyCycle(500);
//		Delay_ms(300);
//		//Delay_ms(2000);
//		Motor_Disable();
	}
}
