#ifndef __LINE_SENSOR_H__
#define __LINE_SENSOR_H__

#include "stm32f10x.h"
#include "BlackPoint_Finder.h"
#include <stdint.h>

extern volatile uint16_t g_line_sensor_values[SENSOR_COUNT];
extern volatile uint16_t g_battery_adc_value;

void LineSensor_Init(void);
uint16_t LineSensor_ReadChannel(uint8_t channel);
void LineSensor_SampleAll(void);

#endif