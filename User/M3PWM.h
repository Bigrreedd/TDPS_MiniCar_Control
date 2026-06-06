#ifndef __M3PWM_H
#define __M3PWM_H

#include "stm32f10x.h"

// PWM相关宏定义
#define PWM_FREQUENCY_17KHZ    17000   // PWM频率17KHz
#define PWM_DUTY_CYCLE_50      50      // 默认占空比50%
/* C0(06-06): 底层无条件硬钳(0~1000 量纲)。SS54FSH 续流二极管 5A + 84A 堵转级风扇电机
 * → D·(1-D)·84A≤4A → D≤5%。任何调用路径(含误指令)物理出不去 5%。
 * 06-05 功率审查的双层钳位只落在 lower-pid(b0c8d62),本分支此前为裸 1000 钳——缺口已补。
 * 解锁三条件(换≥20A二极管/铜皮温升实测/电流采样)见日志 06-05 18:25 / 02_hardware 第7章。 */
#define FAN_DUTY_ABS_CAP       50
/* C3-kick(06-06): 起转诊断构建开关——1=K4 变为单次有界 kick(150/1000=15% 仅 200ms→自动
 * 回落 50 保持),用于判电机死活(17kHz/2kHz 全梯 ≤5% 实测均不起转后启用)。150 只走
 * KickDiag 专用入口,常规 SetDutyCycle 的 50 硬钳不变;点动进行中忽略重按,150 暴露严格
 * ≤200ms(堵转续流 ~10.7A 超二极管额定,只许瞬态)。**诊断结束后置 0 还原 K4 阶梯**。 */
#ifndef FAN_KICK_DIAG_ENABLE
#define FAN_KICK_DIAG_ENABLE   1
#endif

// 函数声明
void M3PWM_Init(void);                    // PWM初始化函数
void M3PWM_SetDutyCycle(uint16_t duty);    // 设置PWM占空比 (0-1000)
void M3PWM_Start(void);                   // 启动PWM输出
void M3PWM_Stop(void);                    // 停止PWM输出
void M3PWM_SetFrequency(uint32_t freq);   // 设置PWM频率

uint16_t M3PWM_GetDutyCycle(void);        // 读取当前占空比设定值（0~1000）
#if FAN_KICK_DIAG_ENABLE
void M3PWM_SetDutyCycleKickDiag(uint16_t duty);  // 诊断专用入口:钳150,仅 K4 kick 200ms 路径调用
#endif

#endif
