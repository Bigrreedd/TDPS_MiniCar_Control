#include "TelemetryScreen.h"
#include "OLED.h"
#include "OLED_CN.h"
#include "BlackPoint_Finder.h"
#include "LineSensor.h"
#include "Motor_ctr.h"
#include "ABEncoder.h"
#include "Battery.h"
#include <stdio.h>

extern volatile float add_angle_deg_360;
extern volatile int16_t position_get;
extern float BDI_V;

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

	int spd_sum = (int)speed_left + (int)speed_right;
	if (spd_sum > 9999)
		spd_sum = 9999;
	if (spd_sum < -9999)
		spd_sum = -9999;

	uint32_t duty = ((uint32_t)Motor_GetDuty(MOTOR_L) + (uint32_t)Motor_GetDuty(MOTOR_R)) / 2u;
	uint8_t pwm_pct = (uint8_t)((duty * 100u) / MOTOR_DUTY_MAX);
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
