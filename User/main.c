#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "Motor_ctr.h"
#include "ABEncoder.h"
#include "Uart_Config.h"
#include "stdio.h"
#include "math.h"
#include "system_stm32f10x.h"
#include "Protocol.h"
#include "stm32f10x_it.h"

/*
 * ============================================================
 *  二合一板 —— 下板（纯执行器 / “脚”）固件
 *  设计目标：烧录一次，永不修改。所有控制算法（位置环/速度环/
 *  Path/起步/丢线/转向）都在上板（大脑），下板只做三件事：
 *    1. 接收上板 MOTOR_CMD（左/右带符号占空比 + 使能）→ 直接驱动 PWM
 *       (DRV8701: PA8/9/10/11 方向+PWM, PA12 使能)
 *    2. 本地编码器测速 → 周期回传 ENC_FEEDBACK 给上板
 *    3. 串口看门狗：上板停发指令(链路断/上板复位)立即停车下电
 *  本板不接灰度/OLED/IMU/按键/风扇，无任何控制逻辑。
 * ============================================================
 */

// 滴答定时器初始化，2ms中断一次 (500Hz)
void SysTick_Init(void)
{
    SysTick_Config(SystemCoreClock / 500);
}

extern volatile uint16_t uart_rx_timeout;   /* 定义于 Motor_ctr.c，串口看门狗计数 */

/* is_racing 在本板含义=电机已使能；保留该符号供 SysTick 看门狗共用 */
volatile uint8_t is_racing = 0;

/* 编码器反馈回传周期：每 N 个 2ms tick 发一帧（5 tick = 10ms = 100Hz） */
#ifndef ENC_FEEDBACK_PERIOD_TICKS
#define ENC_FEEDBACK_PERIOD_TICKS 5u
#endif

/* 大端字节序解码辅助 */
#define PROTO_RD_U16(buf, i) ((int16_t)((uint16_t)(buf)[(i)] << 8 | (buf)[(i)+1]))

// ESP32-S3 雷达数据（保留协议兼容）
volatile uint16_t radar_distance_cm = 0;

/* 把带符号占空比施加到指定电机（正=前进，负=后退） */
static void ApplyMotorDuty(uint8_t motor_id, int16_t duty_signed)
{
    uint8_t direction;
    uint16_t duty;
    if (duty_signed < 0)
    {
        direction = MOTOR_DIR_BACKWARD;
        duty = (uint16_t)(-(int32_t)duty_signed);
    }
    else
    {
        direction = MOTOR_DIR_FORWARD;
        duty = (uint16_t)duty_signed;
    }
    if (duty > MOTOR_DUTY_MAX) duty = MOTOR_DUTY_MAX;
    Motor_SetDirection(motor_id, direction);
    Motor_SetSpeed(motor_id, duty);
}

static void MotorsOffSafe(void)
{
    Motor_StopAll();
    Motor_Disable();
    is_racing = 0;
}

// ========== 上板 -> 下板 电机指令帧（核心） ==========
static void OnMotorCmd(const ProtoFrame_t *f)
{
    if (f->len < PROTO_MOTOR_CMD_LEN) return;

    int16_t duty_l = PROTO_RD_U16(f->payload, 0);
    int16_t duty_r = PROTO_RD_U16(f->payload, 2);
    uint8_t enable = (f->payload[4] & PROTO_MOTOR_FLAG_ENABLE) ? 1u : 0u;

    /* 喂狗：链路有指令 */
    uart_rx_timeout = 0;

    if (enable)
    {
        if (!is_racing)
        {
            Motor_Enable();
            is_racing = 1;
        }
        ApplyMotorDuty(MOTOR_L, duty_l);
        ApplyMotorDuty(MOTOR_R, duty_r);
    }
    else
    {
        MotorsOffSafe();
    }
}

// ========== ESP32 协议回调（保留兼容） ==========
static void OnLoraStop(const ProtoFrame_t *f)
{
    (void)f;
    MotorsOffSafe();
}

static void OnRadarDist(const ProtoFrame_t *f)
{
    if (f->len < 2) return;
    radar_distance_cm = (uint16_t)PROTO_RD_U16(f->payload, 0);
}

// 上板 -> 下板 链路复位：上板刚开机/重烧，立即复位到安全态
static void OnLinkReset(const ProtoFrame_t *f)
{
    (void)f;
    MotorsOffSafe();
    /* 清理里程计数，与上板重新对齐 */
    left_encoder_cnt = 0;
    right_encoder_cnt = 0;
    uart_rx_timeout = 0;
}

int main(void)
{
    // 外设初始化（仅电机/编码器/串口）
    SysTick_Init();
    Motor_Init();
    ABEncoder_Init();
    Uart2_Init(115200);

    // 板间/ESP32 协议初始化
    Proto_Init();
    Proto_RegisterHandler(PROTO_CMD_MOTOR_CMD, OnMotorCmd);
    Proto_RegisterHandler(PROTO_CMD_LORA_STOP, OnLoraStop);
    Proto_RegisterHandler(PROTO_CMD_RADAR_DIST, OnRadarDist);
    Proto_RegisterHandler(PROTO_CMD_LINK_RESET, OnLinkReset);

    Motor_StopAll();
    Motor_Disable();

    /* 下板开机/重烧：通告上板解除运行+复位PID，防止下板刚上电
     * 就被上板陈旧/饱和的 PID 输出猛冲。多发几帧防丢包。 */
    Proto_SendLinkReset();
    Delay_ms(5);
    Proto_SendLinkReset();

    uint32_t fb_last_tick = add_angle_num;

    while (1)
    {
        // ---- 周期回传编码器反馈给上板 ----
        {
            uint32_t now_tick = add_angle_num;
            if ((uint32_t)(now_tick - fb_last_tick) >= ENC_FEEDBACK_PERIOD_TICKS)
            {
                fb_last_tick = now_tick;
                Proto_SendEncFeedback(speed_left, speed_right,
                                      left_encoder_cnt, right_encoder_cnt);
            }
        }

        // ---- 板间协议接收处理（解析 MOTOR_CMD 并即时驱动电机） ----
        Proto_Process();
    }
}
