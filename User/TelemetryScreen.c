#include "TelemetryScreen.h"
#include "OLED.h"
#include "OLED_CN.h"
#include "BlackPoint_Finder.h"
#include "LineSensor.h"
#include "Battery.h"
#include <stdio.h>

extern volatile float add_angle_deg_360;
extern volatile int16_t position_get;
extern float BDI_V;

/* 二合一板：下板经 MOTOR_STATUS 帧回传的电机数据（定义于上板 main.c） */
extern volatile int16_t g_link_spd_l;
extern volatile int16_t g_link_spd_r;
extern volatile uint8_t g_link_pwm_pct;

void TelemetryScreen_Init(void)
{
}

void TelemetryScreen_Update(void)
{
	char pbuf[8];
	char bbuf[8];
	uint8_t i;
	uint16_t yaw_deg = (uint16_t)add_angle_deg_360;
	if (yaw_deg >= 360u)
		yaw_deg = 0u;

	int pos = (int)position_get;
	if (pos > 99999)
		pos = 99999;
	if (pos < -99999)
		pos = -99999;

	int spd_sum = (int)g_link_spd_l + (int)g_link_spd_r;
	if (spd_sum > 9999)
		spd_sum = 9999;
	if (spd_sum < -9999)
		spd_sum = -9999;

	uint8_t pwm_pct = g_link_pwm_pct;
	if (pwm_pct > 100u)
		pwm_pct = 100;

	uint8_t bat = battery_percent(BDI_V);

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

	OLED_ClearLine(3);
	OLED_CN_DrawGlyph(3, 1, CN_SU);
	OLED_ShowChar(3, 3, ':');
	OLED_ShowSignedNum(3, 4, spd_sum, 4);
	OLED_CN_DrawGlyph(3, 9, CN_GONG);
	OLED_ShowChar(3, 11, ':');
	snprintf(pbuf, sizeof(pbuf), "%u%%", (unsigned)pwm_pct);
	OLED_ShowString(3, 12, pbuf);

	OLED_ClearLine(4);
	OLED_CN_DrawGlyph(4, 1, CN_DIAN);
	OLED_CN_DrawGlyph(4, 3, CN_LIANG);
	OLED_ShowChar(4, 5, ':');
	snprintf(bbuf, sizeof(bbuf), "%u%%", (unsigned)bat);
	OLED_ShowString(4, 6, bbuf);
}
