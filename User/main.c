#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "FanMotor_SafetyLock.h"
#include "Motor_ctr.h"
#include "M3PWM.h"
#include "OLED.h"
#include "ABEncoder.h"
#include "LineSensor.h"
#include "Uart_Config.h"
#include "stdio.h"
#include "math.h"
#include "RGB_Led.h"
#include "MPU6050_Config.h"
#include "pose.h"
#include "Key_Scan.h"
#include "system_stm32f10x.h"
#include "BlackPoint_Finder.h"
#include "PID_Controller.h"
#include "Path.h"
#include "TelemetryScreen.h"
#include "Protocol.h"
#include "Battery.h"
#include "stm32f10x_it.h"

// 滴答定时器初始化，2ms中断一次 (500Hz)
void SysTick_Init(void)
{
    SysTick_Config(SystemCoreClock / 500);
}

float BDI_V = 0;
volatile uint8_t is_racing = 0;
static uint8_t g_imu_init_ok = 0;
static uint8_t g_imu_who_id = 0;

// 控制环状态（从 SysTick 移到主循环）
// position_get 的 extern 声明已在 stm32f10x_it.h 中
BlackPointResult_t result_BlackPoint;
static uint32_t lose_time = 0;

// ESP32-S3 雷达数据
volatile uint16_t radar_distance_cm = 0;

/* 大端字节序解码辅助 */
#define PROTO_RD_U16(buf, i) ((int16_t)((uint16_t)(buf)[(i)] << 8 | (buf)[(i)+1]))
#ifndef UART_TEXT_DEBUG_ENABLE
#define UART_TEXT_DEBUG_ENABLE 1
#endif
#ifndef UART_TELEMETRY_PERIOD_TICKS
#define UART_TELEMETRY_PERIOD_TICKS 100u
#endif
#ifndef UART_RAW_ADC_DEBUG_ENABLE
#define UART_RAW_ADC_DEBUG_ENABLE 1
#endif
#ifndef UART_RAW_ADC_PERIOD_TICKS
#define UART_RAW_ADC_PERIOD_TICKS 100u
#endif
#ifndef OLED_TELEMETRY_ENABLE
#define OLED_TELEMETRY_ENABLE 1
#endif
#ifndef OLED_TELEMETRY_PERIOD_TICKS
#define OLED_TELEMETRY_PERIOD_TICKS 150u
#endif
#ifndef LINE_SENSOR_TEST_ONLY
#define LINE_SENSOR_TEST_ONLY 1
#endif
#ifndef SENSOR_DEBUG_MIN_SPAN
#define SENSOR_DEBUG_MIN_SPAN 80u
#endif
#ifndef SENSOR_DEBUG_THRESHOLD_PERCENT
#define SENSOR_DEBUG_THRESHOLD_PERCENT 35u
#endif

static uint8_t g_sensor_min_index = 0;
static uint8_t g_sensor_max_index = 0;
static uint16_t g_sensor_min_value = 0;
static uint16_t g_sensor_max_value = 0;
static uint16_t g_sensor_span = 0;
static uint16_t g_sensor_low_mask = 0;
static uint16_t g_sensor_high_mask = 0;
static int16_t g_sensor_low_pos10 = -1;
static int16_t g_sensor_high_pos10 = -1;

static uint16_t BuildSensorMask(void)
{
    uint16_t mask = 0;
    uint8_t i;
    for (i = 0; i < SENSOR_COUNT; i++)
    {
        if (BlackPoint_Finder_IsBlackPoint(i, (uint16_t)g_line_sensor_values[i]))
        {
            mask |= (uint16_t)(1u << i);
        }
    }
    return mask;
}

static int16_t CalculateSensorDebugPos10(uint16_t mask, uint8_t low_is_target)
{
    uint8_t i;
    uint32_t weight_sum = 0;
    uint32_t position_sum = 0;
    for (i = 0; i < SENSOR_COUNT; i++)
    {
        uint32_t weight;
        if ((mask & (uint16_t)(1u << i)) == 0u)
        {
            continue;
        }
        if (low_is_target)
        {
            weight = (uint32_t)g_sensor_max_value - (uint32_t)g_line_sensor_values[i];
        }
        else
        {
            weight = (uint32_t)g_line_sensor_values[i] - (uint32_t)g_sensor_min_value;
        }
        weight++;
        weight_sum += weight;
        position_sum += weight * (uint32_t)i * 10u;
    }
    if (weight_sum == 0u)
    {
        return -1;
    }
    return (int16_t)((position_sum + weight_sum / 2u) / weight_sum);
}

static void UpdateSensorDebugSnapshot(void)
{
    uint8_t i;
    uint16_t low_threshold;
    uint16_t high_threshold;
    g_sensor_min_value = 0xFFFFu;
    g_sensor_max_value = 0u;
    g_sensor_low_mask = 0u;
    g_sensor_high_mask = 0u;
    g_sensor_low_pos10 = -1;
    g_sensor_high_pos10 = -1;
    for (i = 0; i < SENSOR_COUNT; i++)
    {
        uint16_t value = (uint16_t)g_line_sensor_values[i];
        if (value < g_sensor_min_value)
        {
            g_sensor_min_value = value;
            g_sensor_min_index = i;
        }
        if (value > g_sensor_max_value)
        {
            g_sensor_max_value = value;
            g_sensor_max_index = i;
        }
    }
    g_sensor_span = g_sensor_max_value - g_sensor_min_value;
    if (g_sensor_span < SENSOR_DEBUG_MIN_SPAN)
    {
        return;
    }
    low_threshold = g_sensor_min_value + (uint16_t)(((uint32_t)g_sensor_span * SENSOR_DEBUG_THRESHOLD_PERCENT) / 100u);
    high_threshold = g_sensor_max_value - (uint16_t)(((uint32_t)g_sensor_span * SENSOR_DEBUG_THRESHOLD_PERCENT) / 100u);
    for (i = 0; i < SENSOR_COUNT; i++)
    {
        uint16_t value = (uint16_t)g_line_sensor_values[i];
        if (value <= low_threshold)
        {
            g_sensor_low_mask |= (uint16_t)(1u << i);
        }
        if (value >= high_threshold)
        {
            g_sensor_high_mask |= (uint16_t)(1u << i);
        }
    }
    g_sensor_low_pos10 = CalculateSensorDebugPos10(g_sensor_low_mask, 1u);
    g_sensor_high_pos10 = CalculateSensorDebugPos10(g_sensor_high_mask, 0u);
}

static int32_t ScaleFloat100(float value)
{
    if (value >= 0.0f)
    {
        return (int32_t)(value * 100.0f + 0.5f);
    }
    return (int32_t)(value * 100.0f - 0.5f);
}

static int32_t ScaleFloat1000(float value)
{
    if (value >= 0.0f)
    {
        return (int32_t)(value * 1000.0f + 0.5f);
    }
    return (int32_t)(value * 1000.0f - 0.5f);
}

static void PrintFixed2(const char *name, int32_t value)
{
    if (value < 0)
    {
        int32_t abs_value = -value;
        printf("%s=-%ld.%02ld", name, (long)(abs_value / 100), (long)(abs_value % 100));
    }
    else
    {
        printf("%s=%ld.%02ld", name, (long)(value / 100), (long)(value % 100));
    }
}

static void PrintFixed3(const char *name, int32_t value)
{
    if (value < 0)
    {
        int32_t abs_value = -value;
        printf("%s=-%ld.%03ld", name, (long)(abs_value / 1000), (long)(abs_value % 1000));
    }
    else
    {
        printf("%s=%ld.%03ld", name, (long)(value / 1000), (long)(value % 1000));
    }
}

#if UART_TEXT_DEBUG_ENABLE
static void SendTextDebugTelemetry(void)
{
    static int32_t last_left_encoder_cnt = 0;
    static int32_t last_right_encoder_cnt = 0;
    int32_t yaw100 = ScaleFloat100(add_angle_deg_360);
    int32_t gx1000 = ScaleFloat1000(MPU6050_data.gx_rads);
    int32_t gy1000 = ScaleFloat1000(MPU6050_data.gy_rads);
    int32_t gz1000 = ScaleFloat1000(MPU6050_data.gz_rads);
    int32_t dist100 = ScaleFloat100(Path_GetTotalDistCm());
    int32_t bat100 = ScaleFloat100(BDI_V);
    int32_t left_cnt = left_encoder_cnt;
    int32_t right_cnt = right_encoder_cnt;
    int32_t left_delta = left_cnt - last_left_encoder_cnt;
    int32_t right_delta = right_cnt - last_right_encoder_cnt;
    uint16_t tim3_cnt = TIM_GetCounter(TIM3);
    uint16_t tim4_cnt = TIM_GetCounter(TIM4);
    uint8_t enc_io = 0;

    uint16_t mask = BuildSensorMask();

    last_left_encoder_cnt = left_cnt;
    last_right_encoder_cnt = right_cnt;
    if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_6)) enc_io |= 0x01u;
    if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_7)) enc_io |= 0x02u;
    if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_6)) enc_io |= 0x04u;
    if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_7)) enc_io |= 0x08u;

    printf("DBG R=%u M=%u SEG=%u FOUND=%u POS=%d ",
           (unsigned)is_racing,
           (unsigned)g_manual_drive_active,
           (unsigned)Path_GetCurrentSegment(),
           (unsigned)result_BlackPoint.found,
           (int)position_get);
    PrintFixed2("YAW", yaw100);
    printf(" ");
    PrintFixed3("GX", gx1000);
    printf(" ");
    PrintFixed3("GY", gy1000);
    printf(" ");
    PrintFixed3("GZ", gz1000);
    printf(" GR=%d,%d,%d SL=%d SR=%d SD=%d EL=%ld ER=%ld EDL=%ld EDR=%ld T3=%u T4=%u EIO=%X DL=%u DR=%u FAN=%u ",
           (int)MPU6050_data.gx,
           (int)MPU6050_data.gy,
           (int)MPU6050_data.gz,
           (int)speed_left,
           (int)speed_right,
           (int)(speed_left - speed_right),
           (long)left_cnt,
           (long)right_cnt,
           (long)left_delta,
           (long)right_delta,
           (unsigned)tim3_cnt,
           (unsigned)tim4_cnt,
           (unsigned)enc_io,
           (unsigned)Motor_GetDuty(MOTOR_L),
           (unsigned)Motor_GetDuty(MOTOR_R),
           (unsigned)FanMotor_SafetyLock_GetDutyCycle());

    PrintFixed2("DIST", dist100);
    printf(" ADC=%04X LM=%04X HM=%04X S=%u:%u,%u:%u,%u LP=%d HP=%d ",
           (unsigned int)mask,
           (unsigned int)g_sensor_low_mask,
           (unsigned int)g_sensor_high_mask,
           (unsigned)g_sensor_min_index,
           (unsigned)g_sensor_min_value,
           (unsigned)g_sensor_max_index,
           (unsigned)g_sensor_max_value,
           (unsigned)g_sensor_span,
           (int)g_sensor_low_pos10,
           (int)g_sensor_high_pos10);
    PrintFixed2("BAT", bat100);
    printf(" KEY=%u%u%u%u IMU=%u/%02X\r\n",
           (unsigned)Key_GetState(KEY_K1),
           (unsigned)Key_GetState(KEY_K2),
           (unsigned)Key_GetState(KEY_K3),
           (unsigned)Key_GetState(KEY_K4),
           (unsigned)g_imu_init_ok,
           (unsigned)g_imu_who_id);
    /* 临时诊断：TIM1 硬件寄存器，定位电机不转 */
    printf("TIMDBG MOE=%u CEN=%u CCER=%04X CCR2=%u CCR4=%u ARR=%u PA12=%u PA9nib=%X PA11nib=%X\r\n",
           (unsigned)((TIM1->BDTR >> 15) & 1u),
           (unsigned)(TIM1->CR1 & 1u),
           (unsigned)TIM1->CCER,
           (unsigned)TIM1->CCR2,
           (unsigned)TIM1->CCR4,
           (unsigned)TIM1->ARR,
           (unsigned)(GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_12)),
           (unsigned)((GPIOA->CRH >> 4) & 0xFu),
           (unsigned)((GPIOA->CRH >> 12) & 0xFu));
}
#endif

#if UART_RAW_ADC_DEBUG_ENABLE
static void SendRawAdcTelemetry(void)
{
    printf("RAW7 %u,%u,%u,%u,%u,%u,%u,PA0=%u\r\n",
           (unsigned)g_line_sensor_values[0],
           (unsigned)g_line_sensor_values[1],
           (unsigned)g_line_sensor_values[2],
           (unsigned)g_line_sensor_values[3],
           (unsigned)g_line_sensor_values[4],
           (unsigned)g_line_sensor_values[5],
           (unsigned)g_line_sensor_values[6],
           (unsigned)g_battery_adc_value);
}
#endif

static void SafetyStopAll(void)
{
    Path_StopRace();
    Motor_StopAll();
    Motor_Disable();
    FanMotor_SafetyLock_ForceOff();
    is_racing = 0;
    g_manual_drive_active = 0;
    g_manual_drive_ticks_remaining = 0;
}

// ========== ESP32 协议回调 ==========
/* 开环自检：两电机正转，固定占空比驱动约 1 秒（由 SysTick 计时自动停）。
 * 上闭环前用它确认电机转向、左右编码器计数方向是否正常。
 * duty 经 Motor_SetSpeed 内部安全上限(2000=20%)裁剪，不会超限烧机。
 * K3=1500(15%)、K4=2000(20%) 两档用于找电机起转门槛。 */
#define OPENLOOP_TEST_DUTY_K3 1500u   /* 15% 满量程(10000) */
#define OPENLOOP_TEST_DUTY_K4 2000u   /* 20% 满量程(10000)，等于安全上限 */
#define OPENLOOP_TEST_TICKS   5000u   /* SysTick 2ms/tick -> 约 10s，方便万用表读数 */
/* GPIO 直接输出测试：绕过 TIM1，把 PA9/PA11/PA12 配成普通推挽并拉高。
 * 按 K3 触发。万用表量到 3.3V = 引脚物理连通；量不到 = 焊接断路。 */
static void GpioDirectTest(void)
{
    GPIO_InitTypeDef gpio;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_11 | GPIO_Pin_12;
    GPIO_Init(GPIOA, &gpio);
    GPIO_SetBits(GPIOA, GPIO_Pin_9 | GPIO_Pin_11 | GPIO_Pin_12);
}

static void StartOpenLoopMotorTest(uint16_t duty)
{
    Motor_Enable();
    Motor_SetDirection(MOTOR_L, MOTOR_DIR_FORWARD);
    Motor_SetDirection(MOTOR_R, MOTOR_DIR_FORWARD);
    Motor_SetSpeed(MOTOR_L, duty);
    Motor_SetSpeed(MOTOR_R, duty);
    /* 先设剩余 tick 再置 active，避免 SysTick 在中途看到 active=1 而 ticks=0 立即停 */
    g_manual_drive_ticks_remaining = OPENLOOP_TEST_TICKS;
    g_manual_drive_active = 1;
}

static void OnLoraSpeed(const ProtoFrame_t *f)
{
    if (f->len < 4) return;
    SafetyStopAll();
}

static void OnLoraStop(const ProtoFrame_t *f)
{
    (void)f;
    SafetyStopAll();
}

static void OnRadarDist(const ProtoFrame_t *f)
{
    if (f->len < 2) return;
    radar_distance_cm = (uint16_t)PROTO_RD_U16(f->payload, 0);
}

int main(void)
{
    // 外设初始化
    RGB_Init();
    OLED_Init();
    g_imu_init_ok = MPU6050_Init();
    g_imu_who_id  = MPU6050_ReadID();
    FanMotor_SafetyLock_Init();
    SysTick_Init();
    Motor_Init();
    /* M3PWM_Init();  PCB2: PB11 改作第7路巡线(S7)，弃用风扇 TIM2_CH4，不再初始化 */
    /* M3PWM_Start(); PCB2: 同上，停用风扇 PWM */
    FanMotor_SafetyLock_ForceOff();
    ABEncoder_Init();
    BlackPoint_Finder_Init();
    LineSensor_Init();
    Key_Scan_Init();
    Uart2_Init(115200);
    PID_Init();
    Path_Init();
    TelemetryScreen_Init();

    // ESP32-S3 协议初始化
    Proto_Init();
    Proto_RegisterHandler(PROTO_CMD_LORA_SPEED, OnLoraSpeed);
    Proto_RegisterHandler(PROTO_CMD_LORA_STOP, OnLoraStop);
    Proto_RegisterHandler(PROTO_CMD_RADAR_DIST, OnRadarDist);

    uint8_t uart_telem_due = 0;
    uint32_t uart_telem_last_tick = add_angle_num;
#if UART_TEXT_DEBUG_ENABLE && UART_RAW_ADC_DEBUG_ENABLE
    uint8_t uart_raw_adc_due = 0;
    uint32_t uart_raw_adc_last_tick = add_angle_num;
#endif
#if OLED_TELEMETRY_ENABLE
    uint8_t oled_due = 1;
    uint32_t oled_last_tick = add_angle_num;
#endif

    while (1)
    {
        // ---- 主循环控制环：由 SysTick 标志位驱动 ----
        if (g_control_tick)
        {
            g_control_tick = 0;

            // 7路光电 + PA0电池电压轮询采样（耗时操作，已从中断移出）
            LineSensor_SampleAll();
            UpdateSensorDebugSnapshot();

            // 黑线识别
            BlackPoint_Finder_Search(g_line_sensor_values, &result_BlackPoint);

            if (result_BlackPoint.found)
            {
                position_get = (int16_t)(result_BlackPoint.precise_position * 10.0f);
                if (lose_time > 0) lose_time--;
            }
            else if (!g_manual_drive_active)
            {
                lose_time++;
                if (lose_time > 500)
                {
                    lose_time = 500;
                    SafetyStopAll();
                }
            }

            // 安全锁状态下不执行闭环电机输出
            if (!g_manual_drive_active)
            {
                SafetyStopAll();
            }

        }

        {
            uint32_t now_tick = add_angle_num;
            if ((uint32_t)(now_tick - uart_telem_last_tick) >= UART_TELEMETRY_PERIOD_TICKS)
            {
                uart_telem_last_tick = now_tick;
                uart_telem_due = 1;
            }
#if UART_TEXT_DEBUG_ENABLE && UART_RAW_ADC_DEBUG_ENABLE
            if ((uint32_t)(now_tick - uart_raw_adc_last_tick) >= UART_RAW_ADC_PERIOD_TICKS)
            {
                uart_raw_adc_last_tick = now_tick;
                uart_raw_adc_due = 1;
            }
#endif
#if OLED_TELEMETRY_ENABLE
            if ((uint32_t)(now_tick - oled_last_tick) >= OLED_TELEMETRY_PERIOD_TICKS)
            {
                oled_last_tick = now_tick;
                oled_due = 1;
            }
#endif
        }

        // ---- ESP32-S3 协议处理（非实时，低优先级） ----
        Proto_Process();

        // ---- 人机交互（非实时，低优先级） ----
        Key_Scan_Update();
        Key_Event_t *event = Key_GetEvent();
        BDI_V = (float)g_battery_adc_value * 0.00426508726f;

        if (event != NULL)
        {
            switch (event->key_id)
            {
            case KEY_NONE:
                break;
            case KEY_K1:
                // 认键模式：仅显示，不驱动电机
                SafetyStopAll();
                lose_time = 0;
                BlackPoint_Finder_ResetLastPosition();
                OLED_ShowString(1, 1, "KEY=K1          ");
                break;
            case KEY_K2:
                // 认键模式：仅显示，不驱动电机
                SafetyStopAll();
                lose_time = 0;
                OLED_ShowString(1, 1, "KEY=K2          ");
                break;
            case KEY_K3:
                // K3: 开环自检 15%（轮子正转约1秒，SysTick 自动停）
                OLED_ShowString(1, 1, "KEY=K3 15%      ");
                StartOpenLoopMotorTest(OPENLOOP_TEST_DUTY_K3);
                break;
            case KEY_K4:
                // K4: 开环自检 20%（轮子正转约1秒，SysTick 自动停）
                OLED_ShowString(1, 1, "KEY=K4 20%      ");
                StartOpenLoopMotorTest(OPENLOOP_TEST_DUTY_K4);
                break;
            }
        }

#if OLED_TELEMETRY_ENABLE
        if (oled_due)
        {
            oled_due = 0;
            if (!is_racing && !g_manual_drive_active)
            {
                TelemetryScreen_Update();
            }
        }
#endif

        if (uart_telem_due)
        {
            uart_telem_due = 0;
#if UART_TEXT_DEBUG_ENABLE
            SendTextDebugTelemetry();
#else
            Proto_SendTelemetry(position_get, speed_left, speed_right,
                                (uint8_t)Path_GetCurrentSegment(), battery_percent(BDI_V));
#endif
        }

#if UART_TEXT_DEBUG_ENABLE && UART_RAW_ADC_DEBUG_ENABLE
        if (uart_raw_adc_due)
        {
            uart_raw_adc_due = 0;
            SendRawAdcTelemetry();
        }
#endif
    }
}