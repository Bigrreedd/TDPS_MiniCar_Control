#ifndef __ABENCODER_H
#define __ABENCODER_H

#include "stm32f10x.h"

void ABEncoder_Init(void);
void ABEncoder_UpdateSpeed(void);
extern volatile int16_t speed_left, speed_right;
extern volatile int32_t left_encoder_cnt, right_encoder_cnt;
#endif