#include "TelemetryScreen.h"
#include "OLED.h"
#include "OLED_CN.h"
#include "BlackPoint_Finder.h"
#include "ADC_get.h"
#include "M3PWM.h"
#include "ABEncoder.h"
#include <stdio.h>

#define RAD_TO_DEG (57.2957795f)

#define TELEM_BATTERY_V_MIN 5.5f
#define TELEM_BATTERY_V_MAX 8.6f

extern float add_angle;
extern int16_t position_get;
extern float BDI_V;

static float s_angle_ref_rad;

static uint8_t battery_percent(float v)
{
	if (v <= TELEM_BATTERY_V_MIN)
		return 0;
	if (v >= TELEM_BATTERY_V_MAX)
		return 100;
	return (uint8_t)((v - TELEM_BATTERY_V_MIN) * 100.0f / (TELEM_BATTERY_V_MAX - TELEM_BATTERY_V_MIN));
}

static uint16_t build_sensor_mask(void)
{
	uint16_t m = 0;
	uint8_t i;
	for (i = 0; i < SENSOR_COUNT; i++)
	{
		if (BlackPoint_Finder_IsBlackPoint(i, (uint16_t)g_mux_adc_values[i]))
			m |= (uint16_t)(1u << i);
	}
	return m;
}

void TelemetryScreen_Init(void)
{
	s_angle_ref_rad = add_angle;
}

void TelemetryScreen_Update(void)
{
	char ubuf[8];
	char pbuf[8];
	char bbuf[8];
	float dy_deg = (add_angle - s_angle_ref_rad) * RAD_TO_DEG;
	int yaw_deg = (int)(dy_deg >= 0.0f ? (dy_deg + 0.5f) : (dy_deg - 0.5f));
	if (yaw_deg > 999)
		yaw_deg = 999;
	if (yaw_deg < -999)
		yaw_deg = -999;

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

	int v100 = (int)(BDI_V * 100.0f + 0.5f);
	if (v100 < 0)
		v100 = 0;

	uint16_t duty = M3PWM_GetDutyCycle();
	uint8_t pwm_pct = (uint8_t)((duty * 100u) / 1000u);
	if (pwm_pct > 100u)
		pwm_pct = 100;

	uint8_t bat = battery_percent(BDI_V);

	uint16_t mask = build_sensor_mask();

	OLED_ClearLine(1);
	OLED_CN_DrawGlyph(1, 1, CN_WEI);
	OLED_ShowChar(1, 3, ':');
	OLED_ShowSignedNum(1, 4, pos, 5);
	OLED_CN_DrawGlyph(1, 10, CN_JIAO);
	OLED_ShowChar(1, 12, ':');
	OLED_ShowSignedNum(1, 13, yaw_deg, 3);

	snprintf(ubuf, sizeof(ubuf), "%2d.%02dV", v100 / 100, v100 % 100);
	OLED_ClearLine(2);
	OLED_CN_DrawGlyph(2, 1, CN_YA);
	OLED_ShowChar(2, 3, ':');
	OLED_ShowString(2, 4, ubuf);
	OLED_CN_DrawGlyph(2, 10, CN_GUANG);
	OLED_ShowChar(2, 12, ':');
	OLED_ShowHexNum(2, 13, (uint32_t)mask, 4);

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
