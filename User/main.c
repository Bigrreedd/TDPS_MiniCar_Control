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
#include "Key_Scan.h"

/* LOWER_PID_TUNE 开关定义在 stm32f10x_it.h（main 与 SysTick 共用同一值）。
 *   =1 速度环脱机调参：开机跑速度环，串口(USART2,115200) ASCII 命令调
 *      Kp/Ki/Kd/目标速度并打印遥测；口径与上板一致，参数可直接搬到上板。
 *      下发链路 = 限幅(±HARD_CAP) -> 加左右死区前馈 -> 限幅(±FINAL_CAP)。
 *      调参务必架空轮子：脱机无看门狗，使能后电机持续转。
 *   =0 原纯执行器固件（收 MOTOR_CMD 驱动 PWM + 回传编码器 + 看门狗）。 */

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

#if !LOWER_PID_TUNE
/* 以下纯执行器回调仅在非调参模式编译，避免 tune 模式下未引用的 static 函数告警 */

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
#endif /* !LOWER_PID_TUNE */

#if LOWER_PID_TUNE
/* ==================== 速度环脱机调参 harness ==================== */

/* 口径常量：与上板逐一对齐 */
#define TUNE_SPEED_WIN_MS    100u     /* 低速诊断窗口；100ms 下 1 tick = 10 cnt/s */
#define TUNE_HARD_CAP        1000.0f  /* PID 调节量限幅，= 上板 MOTOR_DUTY_HARD_CAP */
#define TUNE_DEADZONE_L      750.0f   /* 左轮起步死区，= 上板 MOTOR_DEADZONE_L */
#define TUNE_DEADZONE_R      950.0f   /* 右轮起步死区，= 上板 MOTOR_DEADZONE_R */
#define TUNE_CMD_EPS         1.0f
#define TUNE_FINAL_CAP       (TUNE_HARD_CAP + TUNE_DEADZONE_R)  /* 最终安全上限 1950 */
#define TUNE_PRINT_MS        100u     /* 遥测打印周期(ms)，与测速窗口同步 */
#define TUNE_MIN_TARGET      70.0f    /* 低于此速度容易跨不过起步死区 */
#define TUNE_DEFAULT_TARGET  70.0f    /* 无串口时，按 K1 默认跑 70 cnt/s */
#define TUNE_TARGET_STEP     10.0f    /* K3/K4 每次加减目标速度 */

typedef struct
{
    float kp;
    float ki;
    float kd;
    float output_max;
    float output_min;
    float last_error;
    float last_last_error;
    float last_output;
} TuneSpeedPID_t;

static TuneSpeedPID_t g_tune_pid;

static void tune_pid_init(TuneSpeedPID_t *pid, float kp, float ki, float kd, float out_max, float out_min)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->output_max = out_max;
    pid->output_min = out_min;
    pid->last_error = 0.0f;
    pid->last_last_error = 0.0f;
    pid->last_output = 0.0f;
}

static void tune_pid_reset(TuneSpeedPID_t *pid)
{
    pid->last_error = 0.0f;
    pid->last_last_error = 0.0f;
    pid->last_output = 0.0f;
}

static float tune_pid_calc(TuneSpeedPID_t *pid, float target, float current)
{
    float error = target - current;
    float p = pid->kp * (error - pid->last_error);
    float i = pid->ki * error;
    float d = pid->kd * ((error - pid->last_error) - (pid->last_error - pid->last_last_error));
    float out = pid->last_output + p + i + d;

    if (out > pid->output_max) out = pid->output_max;
    else if (out < pid->output_min) out = pid->output_min;

    pid->last_last_error = pid->last_error;
    pid->last_error = error;
    pid->last_output = out;
    return out;
}

static void tune_apply_motor(uint8_t motor_id, float duty_signed)
{
    uint8_t dir = MOTOR_DIR_FORWARD;
    uint16_t duty;
    if (duty_signed < 0.0f)
    {
        dir = MOTOR_DIR_BACKWARD;
        duty_signed = -duty_signed;
    }
    if (duty_signed > (float)MOTOR_DUTY_MAX) duty_signed = (float)MOTOR_DUTY_MAX;
    duty = (uint16_t)duty_signed;
    Motor_SetDirection(motor_id, dir);
    Motor_SetSpeed(motor_id, duty);
}

static float tune_clamp(float v, float cap)
{
    if (v >  cap) return  cap;
    if (v < -cap) return -cap;
    return v;
}

static float tune_deadzone(float duty, float dz)
{
    if (duty >  TUNE_CMD_EPS) return duty + dz;
    if (duty < -TUNE_CMD_EPS) return duty - dz;
    return 0.0f;
}

/* 打印 float 的千分位，避开 Keil microlib 常见的 printf 浮点支持问题 */
static int32_t tune_float_milli(float v)
{
    if (v >= 0.0f) return (int32_t)(v * 1000.0f + 0.5f);
    return (int32_t)(v * 1000.0f - 0.5f);
}

static void tune_print_float3(float v)
{
    int32_t m = tune_float_milli(v);
    if (m < 0)
    {
        printf("-");
        m = -m;
    }
    printf("%ld.%03ld", (long)(m / 1000), (long)(m % 1000));
}

/* 轻量小数解析：支持 -12.345，避免引入 atof */
static float tune_parse_float(const char *s)
{
    int sign = 1;
    int32_t ip = 0;
    int32_t fp = 0;
    int32_t scale = 1;
    if (*s == '-')
    {
        sign = -1;
        s++;
    }
    else if (*s == '+')
    {
        s++;
    }
    while (*s >= '0' && *s <= '9')
    {
        ip = ip * 10 + (*s - '0');
        s++;
    }
    if (*s == '.')
    {
        s++;
        while (*s >= '0' && *s <= '9')
        {
            fp = fp * 10 + (*s - '0');
            scale *= 10;
            s++;
        }
    }
    return (float)sign * ((float)ip + (float)fp / (float)scale);
}

static void tune_start(uint8_t *p_run)
{
    tune_pid_reset(&g_tune_pid);
    Motor_Enable();
    *p_run = 1u;
    printf("OK run\r\n");
}

static void tune_stop(uint8_t *p_run)
{
    *p_run = 0u;
    Motor_StopAll();
    Motor_Disable();
    tune_pid_reset(&g_tune_pid);
    printf("OK stop\r\n");
}

static float tune_limit_target(float target)
{
    if (target < TUNE_MIN_TARGET) return TUNE_MIN_TARGET;
    return target;
}

static void tune_print_target(float target)
{
    printf("OK target=%d cnt/s\r\n", (int)target);
}

static void tune_handle_key_event(float *p_target, uint8_t *p_run)
{
    Key_Event_t *ev;
    Key_Scan_Update();
    while (Key_HasEvent())
    {
        ev = Key_GetEvent();
        if (ev == 0) return;
        printf("KEY=%d\r\n", (int)ev->key_id);
        switch (ev->key_id)
        {
        case KEY_K1:
            if (*p_run) tune_stop(p_run);
            else tune_start(p_run);
            break;
        case KEY_K2:
            tune_stop(p_run);
            break;
        case KEY_K3:
            *p_target += TUNE_TARGET_STEP;
            tune_print_target(*p_target);
            break;
        case KEY_K4:
            *p_target = tune_limit_target(*p_target - TUNE_TARGET_STEP);
            tune_print_target(*p_target);
            break;
        default:
            break;
        }
    }
}

/* 解析一行命令：p/i/d 设增益, t 设目标速度(计数/秒), g 启动, s 停车, ? 查看 */
static void tune_handle_line(char *line, float *p_target, uint8_t *p_run)
{
    char c = line[0];
    float val = tune_parse_float(line + 1);   /* 命令字母后的数值，无则为 0 */
    switch (c)
    {
    case 'p': case 'P':
        g_tune_pid.kp = val;
        printf("OK Kp=");
        tune_print_float3(val);
        printf("\r\n");
        break;
    case 'i': case 'I':
        g_tune_pid.ki = val;
        printf("OK Ki=");
        tune_print_float3(val);
        printf("\r\n");
        break;
    case 'd': case 'D':
        g_tune_pid.kd = val;
        printf("OK Kd=");
        tune_print_float3(val);
        printf("\r\n");
        break;
    case 't': case 'T':
        *p_target = tune_limit_target(val);
        tune_print_target(*p_target);
        break;
    case 'g': case 'G':
        tune_start(p_run);
        break;
    case 's': case 'S':
        tune_stop(p_run);
        break;
    case '?':
        printf("Kp=");
        tune_print_float3(g_tune_pid.kp);
        printf(" Ki=");
        tune_print_float3(g_tune_pid.ki);
        printf(" Kd=");
        tune_print_float3(g_tune_pid.kd);
        printf(" target=%d run=%d\r\n", (int)*p_target, (int)*p_run);
        break;
    default:
        /* 空行/未知命令忽略 */
        break;
    }
}

/* 从串口环形缓冲非阻塞读取，攒到换行(\r 或 \n)再解析一行 */
static void tune_poll_uart(float *p_target, uint8_t *p_run)
{
    static char buf[32];
    static uint8_t idx = 0u;
    while (Uart2_BytesAvailable() > 0)
    {
        uint8_t ch = Uart2_ReadByteBlocking();
        if (ch == '\r' || ch == '\n')
        {
            if (idx > 0u)
            {
                buf[idx] = '\0';
                tune_handle_line(buf, p_target, p_run);
                idx = 0u;
            }
        }
        else if (idx < (uint8_t)(sizeof(buf) - 1u))
        {
            buf[idx++] = (char)ch;
        }
        /* 超长行：丢弃多余字节直到换行，避免越界 */
    }
}

int main(void)
{
    float target_speed = TUNE_DEFAULT_TARGET;  /* 目标速度(cnt/s)，K3/K4 或串口 t 命令可改 */
    uint8_t run = 0u;                /* 1=速度环运行中 */
    int32_t last_cnt_l = 0, last_cnt_r = 0;
    int32_t vel_accum_l = 0, vel_accum_r = 0;
    uint32_t vel_t0;
    uint32_t print_t0;
    int16_t spd_l = 0, spd_r = 0;    /* 诊断显示：窗口换算后的 cnt/s */
    int16_t raw_l = 0, raw_r = 0;    /* legacy口径：SysTick 2ms内编码器增量 */
    int32_t win_ticks_l = 0, win_ticks_r = 0;
    uint32_t win_dt = 0u;
    float duty_cmd = 0.0f;           /* 速度环输出(死区前) */
    uint32_t now;
    int32_t cnt_l, cnt_r;
    uint32_t dt;
    float avg;
    float dl, dr;

    SysTick_Init();
    Motor_Init();
    ABEncoder_Init();
    Uart2_Init(115200);
    Key_Scan_Init();
    Key_ClearEvent();

    /* 速度环：初值用上板调好的增益，串口可改；输出上限用 HARD_CAP(调节量) */
    tune_pid_init(&g_tune_pid, 12.0f, 2.5f, 0.0f, TUNE_HARD_CAP, -TUNE_HARD_CAP);

    Motor_StopAll();
    Motor_Disable();

    vel_t0 = Millis_Get();
    print_t0 = vel_t0;
    last_cnt_l = left_encoder_cnt;
    last_cnt_r = right_encoder_cnt;

    printf("\r\n=== LOWER PID TUNE (speed loop) ===\r\n");
    printf("keys: K1 run/stop, K2 stop, K3 target+10, K4 target-10\r\n");
    printf("uart: p<Kp> i<Ki> d<Kd> t<target cnt/s> g(run) s(stop) ?(show)\r\n");
    printf("default target=%d cnt/s; wheels off ground! no watchdog in tune mode.\r\n", (int)target_speed);

    while (1)
    {
        now = Millis_Get();

        /* 1. 按键/串口命令轮询 */
        tune_handle_key_event(&target_speed, &run);
        tune_poll_uart(&target_speed, &run);

        raw_l = speed_left;
        raw_r = speed_right;

        /* 2. 诊断速度窗口：累计编码器计数并换算为 cnt/s */
        cnt_l = left_encoder_cnt;
        cnt_r = right_encoder_cnt;
        vel_accum_l += (cnt_l - last_cnt_l);
        vel_accum_r += (cnt_r - last_cnt_r);
        last_cnt_l = cnt_l;
        last_cnt_r = cnt_r;

        if ((uint32_t)(now - vel_t0) >= TUNE_SPEED_WIN_MS)
        {
            dt = now - vel_t0;
            win_dt = dt;
            win_ticks_l = vel_accum_l;
            win_ticks_r = vel_accum_r;
            spd_l = (int16_t)((win_ticks_l * 1000) / (int32_t)dt);
            spd_r = (int16_t)((win_ticks_r * 1000) / (int32_t)dt);
            vel_accum_l = 0;
            vel_accum_r = 0;
            vel_t0 = now;

            /* 3. 有新诊断窗口才算速度环；控制反馈使用窗口换算后的 cnt/s */
            if (run)
            {
                avg = ((float)spd_l + (float)spd_r) * 0.5f;
                duty_cmd = tune_pid_calc(&g_tune_pid, target_speed, avg);

                /* 左右用同一速度环输出，各自加自己的死区前馈 */
                dl = tune_clamp(tune_deadzone(tune_clamp(duty_cmd, TUNE_HARD_CAP), TUNE_DEADZONE_L), TUNE_FINAL_CAP);
                dr = tune_clamp(tune_deadzone(tune_clamp(duty_cmd, TUNE_HARD_CAP), TUNE_DEADZONE_R), TUNE_FINAL_CAP);
                tune_apply_motor(MOTOR_L, dl);
                tune_apply_motor(MOTOR_R, dr);
            }
        }

        /* 4. 周期打印遥测：T/RAW 为 legacy 2ms增量口径，CPS 为窗口换算值 */
        if ((uint32_t)(now - print_t0) >= TUNE_PRINT_MS)
        {
            print_t0 = now;
            if (run)
                printf("T=%d RAWL=%d RAWR=%d CPSL=%d CPSR=%d out=%d tickL=%ld tickR=%ld dt=%lu\r\n",
                       (int)target_speed, (int)raw_l, (int)raw_r, (int)spd_l, (int)spd_r, (int)duty_cmd,
                       (long)win_ticks_l, (long)win_ticks_r, (unsigned long)win_dt);
        }
    }
}

#else  /* ===== LOWER_PID_TUNE=0：原纯执行器固件 ===== */

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

#endif /* LOWER_PID_TUNE */
