#include "stm32f10x.h"
#include "ABEncoder.h"

static int16_t last_count_left = 0, last_count_right = 0;
volatile int16_t speed_left = 0, speed_right = 0;
volatile int32_t left_encoder_cnt = 0, right_encoder_cnt = 0;

static void Encoder_GPIO_Init(GPIO_TypeDef *GPIOx)
{
    GPIO_InitTypeDef gpio;
    gpio.GPIO_Pin   = GPIO_Pin_6 | GPIO_Pin_7;
    gpio.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOx, &gpio);
}

static void Encoder_TIM_Init(TIM_TypeDef *TIMx)
{
    TIM_TimeBaseInitTypeDef tim;
    tim.TIM_Period        = 0xFFFF;
    tim.TIM_Prescaler     = 0;
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    tim.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIMx, &tim);

    TIM_EncoderInterfaceConfig(TIMx, TIM_EncoderMode_TI12,
                               TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
    TIM_ClearFlag(TIMx, TIM_FLAG_Update);
    TIM_SetCounter(TIMx, 0);
    TIM_Cmd(TIMx, ENABLE);
}

void ABEncoder_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4, ENABLE);

    Encoder_GPIO_Init(GPIOA);   // PA6/PA7 → TIM3 右轮编码器
    Encoder_GPIO_Init(GPIOB);   // PB6/PB7 → TIM4 左轮编码器

    Encoder_TIM_Init(TIM3);     // 右轮编码器 (读数取负)
    Encoder_TIM_Init(TIM4);     // 左轮编码器
}

void ABEncoder_UpdateSpeed(void) // 计算速度
{
    /* 编码器极性：使车体前进时 speed_left/speed_right 均为正
       原代码左编码器取负、右编码器不取负，实测 K3/K4 单边前进时
       两边的 EDL/EDR 都为负，会导致 PID 速度反馈正反馈，调不出直线 */
    int16_t now_left  = (int16_t)TIM_GetCounter(TIM4);   // 左编码器：前进 → 计数增加
    int16_t now_right = -(int16_t)TIM_GetCounter(TIM3);  // 右编码器：硬件方向相反，软件取反
    // 先用 uint32_t 做差再截断为 int16_t，正确处理 0/65535 边界穿越
    speed_left = (int16_t)((uint32_t)(uint16_t)now_left - (uint32_t)(uint16_t)last_count_left);
    speed_right = (int16_t)((uint32_t)(uint16_t)now_right - (uint32_t)(uint16_t)last_count_right);
    last_count_left = now_left;
    last_count_right = now_right;
    left_encoder_cnt += speed_left;
    right_encoder_cnt += speed_right;
}