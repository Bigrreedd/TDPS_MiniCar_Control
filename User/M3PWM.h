#ifndef __M3PWM_H
#define __M3PWM_H

#include "stm32f10x.h"

// PWM相关宏定义
#define PWM_FREQUENCY_17KHZ    17000   // PWM频率17KHz
#define PWM_DUTY_CYCLE_50      50      // 默认占空比50%
/* C0(06-06): 底层无条件硬钳(0~1000 量纲)。原值 50 按单颗 SS54FSH 5A 降额 4A:
 * D·(1-D)·84A≤4A → D≤5%。06-05 双层钳位只落在 lower-pid(b0c8d62),本分支缺口已补。
 * G3(06-07 用户拍板): 50→100,板级堵转仍全额定内(19:00 网表实证三颗 SS54FSH 并联,
 * 降额 ~10A: 堵转@10% 二极管 7.6A 总=2.5A/颗 ✓ XT30 8.4A<15A ✓ NMOS 无压力),
 * 即永久堵转也烧不了板,唯一牺牲件=风扇电机自身(卡死本就报废)。台架 60s 摸温过测后
 * 与 FAN_RACE_HOLD_DUTY=100 一并锁死;不过测双双回 50。电流采样仍缺,程序性替代=
 * 每轮 kick 可闻起转自检+人跟车 K2。 */
#define FAN_DUTY_ABS_CAP       100
/* C3-kick(06-06): 起转诊断构建开关——1=K4 变为单次有界 kick(150/1000=15% 仅 200ms→自动
 * 回落 50 保持),用于判电机死活(17kHz/2kHz 全梯 ≤5% 实测均不起转后启用)。150 只走
 * KickDiag 专用入口,常规 SetDutyCycle 的 50 硬钳不变;点动进行中忽略重按,150 暴露严格
 * ≤200ms(堵转续流 ~10.7A 超二极管额定,只许瞬态;19:00 网表实证三颗 SS54FSH 并联,
 * 每颗 ~3.6A 额定内)。G2(06-07) 起 G1 锁存/K1 发车自动起扇均依赖本入口,**此开关
 * 保持 1**(置 0 仅还原 C1 阶梯,会退役 G1/G2 并在其调用点编译失败)。 */
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
