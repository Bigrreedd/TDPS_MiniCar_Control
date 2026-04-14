#ifndef __TELEMETRY_SCREEN_H__
#define __TELEMETRY_SCREEN_H__

#include "stm32f10x.h"

/**
 * @brief 上电后调用一次：记录当前陀螺积分角 add_angle 作为基准，用于 dY（°）显示。
 */
void TelemetryScreen_Init(void);

/**
 * @brief 主循环中周期调用：刷新 4×16 OLED（P/dY、U/h、E/P、Bat 等，详见 Software/log.md）。
 */
void TelemetryScreen_Update(void);

#endif
