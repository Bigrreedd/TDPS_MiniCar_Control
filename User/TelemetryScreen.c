#include "TelemetryScreen.h"
#include "OLED.h"
#include "OLED_CN.h"
#include "BlackPoint_Finder.h"
#include "LineSensor.h"
#include "ABEncoder.h"
#include "Motor_ctr.h"
#include "Path.h"
#include <stdio.h>

extern volatile float add_angle_deg_360;
extern volatile int16_t position_get;

/* 二合一板上板：轮速来自下板 ENC_FEEDBACK(写入 speed_left/right)，
 * 电机目标占空比来自本地控制算法 g_motor_target_l/r */
extern volatile float g_motor_target_l;
extern volatile float g_motor_target_r;

void TelemetryScreen_Init(void)
{
}

void TelemetryScreen_Update(void)
{
	char pbuf[8];
	uint8_t i;
	int spd_l, spd_r;
	uint16_t yaw_deg = (uint16_t)add_angle_deg_360;
	if (yaw_deg >= 360u)
		yaw_deg = 0u;

	int pos = (int)position_get;
	if (pos > 99999)
		pos = 99999;
	if (pos < -99999)
		pos = -99999;

	/* 左右轮速分别显示，限幅 ±999 适配 3 位 */
	spd_l = (int)speed_left;
	if (spd_l > 999) spd_l = 999;
	if (spd_l < -999) spd_l = -999;
	spd_r = (int)speed_right;
	if (spd_r > 999) spd_r = 999;
	if (spd_r < -999) spd_r = -999;

	float duty_avg = (g_motor_target_l + g_motor_target_r) * 0.5f;
	if (duty_avg < 0.0f) duty_avg = -duty_avg;
	uint8_t pwm_pct = (uint8_t)((duty_avg * 100.0f) / (float)MOTOR_DUTY_MAX);
	if (pwm_pct > 100u)
		pwm_pct = 100;

	OLED_ClearLine(1);
	OLED_CN_DrawGlyph(1, 1, CN_WEI);
	OLED_ShowChar(1, 3, ':');
	OLED_ShowSignedNum(1, 4, pos, 5);
	OLED_CN_DrawGlyph(1, 10, CN_JIAO);
	OLED_ShowChar(1, 12, ':');
	OLED_ShowNum(1, 13, yaw_deg, 3);

	OLED_ClearLine(2);
	OLED_CN_DrawGlyph(2, 1, CN_GUANG);
	OLED_ShowChar(2, 3, ':');
	for (i = 0; i < SENSOR_COUNT; i++)
		OLED_ShowChar(2, (uint8_t)(4 + i * 2), (g_line_sensor_values[i] != 0u) ? '1' : '0');

	/* 第 3 行：左右轮实测速度  L:±xxx R:±xxx (计数/秒) */
	OLED_ClearLine(3);
	OLED_ShowChar(3, 1, 'L');
	OLED_ShowChar(3, 2, ':');
	OLED_ShowSignedNum(3, 3, spd_l, 3);
	OLED_ShowChar(3, 8, 'R');
	OLED_ShowChar(3, 9, ':');
	OLED_ShowSignedNum(3, 10, spd_r, 3);

	/* 第 4 行：目标速度 + 当前 PWM 占空比  T:xxx D:xxx%
	 * 与第 3 行实测对比 → 速度环是否把实测拉到目标；D% 看 PID 输出量级 */
	{
		int tgt = (int)Path_GetTargetSpeed();
		if (tgt > 999) tgt = 999; else if (tgt < -999) tgt = -999;
		OLED_ClearLine(4);
		OLED_ShowChar(4, 1, 'T');
		OLED_ShowChar(4, 2, ':');
		OLED_ShowSignedNum(4, 3, tgt, 3);
		OLED_ShowChar(4, 9, 'D');
		OLED_ShowChar(4, 10, ':');
		OLED_ShowNum(4, 11, pwm_pct, 3);
		OLED_ShowChar(4, 14, '%');
	}
	(void)pbuf;
}
