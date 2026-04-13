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

// 滴答定时器初始化，1ms中断一次
void SysTick_Init(void)
{
    // 配置SysTick为1ms中断（SystemCoreClock / 1000）
    SysTick_Config(SystemCoreClock / 500);
}
extern float add_angle ;
extern float add_angle_num ;
extern int16_t position_get;
extern BlackPointResult_t result_BlackPoint;
float BDI_V = 0;
uint8_t star_car = 0;
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
//	//串口2初始化
	 Uart2_Init(115200);
	 PID_Init();
//	Delay_s(5);
	/*主循环，循环体内的代码会一直循环执行*/
	while (1)
	{
	//	LSM6DSR_ReadData(&LSE6DSR_data);
		Key_Scan_Update();
    // 处理按键事件
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
							Motor_Enable();
							M3PWM_SetDutyCycle(950);
							star_car = 1;
                break;
            case KEY_K2:
							RGB_SetColor(2);
                break;
						case KEY_K3:
							RGB_SetColor(4);
                break;
						case KEY_K4:
							RGB_SetColor(5);
                break;
        }
    }

	  OLED_ShowSignedNum(2,1,position_get,5);
		OLED_ShowSignedNum(3,1,BDI_V * 100,5);
//		OLED_ShowSignedNum(3,1,result_BlackPoint.found,5);
		OLED_ShowSignedNum(4,1,speed_left + speed_right,5);
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
