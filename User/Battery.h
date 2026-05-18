#ifndef __BATTERY_H__
#define __BATTERY_H__

#include <stdint.h>

#define BATT_V_MIN 5.5f
#define BATT_V_MAX 8.6f

static inline uint8_t battery_percent(float v)
{
    if (v <= BATT_V_MIN) return 0;
    if (v >= BATT_V_MAX) return 100;
    return (uint8_t)((v - BATT_V_MIN) * 100.0f / (BATT_V_MAX - BATT_V_MIN));
}

#endif /* __BATTERY_H__ */
