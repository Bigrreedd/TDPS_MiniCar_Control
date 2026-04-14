#ifndef __OLED_CN_H__
#define __OLED_CN_H__

#include "stm32f10x.h"

enum
{
	CN_WEI = 0,
	CN_JIAO = 1,
	CN_YA = 2,
	CN_GUANG = 3,
	CN_SU = 4,
	CN_GONG = 5,
	CN_DIAN = 6,
	CN_LIANG = 7
};

void OLED_CN_DrawGlyph(uint8_t line, uint8_t start_column, uint8_t idx);

#endif
