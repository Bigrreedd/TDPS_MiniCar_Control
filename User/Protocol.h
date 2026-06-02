#ifndef __PROTOCOL_H__
#define __PROTOCOL_H__

#include "stm32f10x.h"
#include <stdint.h>

/*
 * ESP32-S3 <-> STM32 UART 协议
 *
 * 帧格式: [0xAA][LEN][CMD][PAYLOAD 0..64 bytes][CRC8]
 *   - 0xAA: 帧头
 *   - LEN:  payload 长度 (0-64)
 *   - CMD:  命令字节
 *   - CRC8: LEN ^ CMD ^ payload[0] ^ ... ^ payload[LEN-1]
 *
 * ESP32 -> STM32 命令:
 *   0x01 LORA_SPEED   : [left_hi][left_lo][right_hi][right_lo]  (int16 占空比)
 *   0x02 LORA_STOP    : 无 payload
 *   0x03 RADAR_DIST   : [dist_hi][dist_lo]  (uint16, cm)
 *   0x10 PING         : 无 payload
 *
 * 二合一板 上板(大脑) -> 下板(执行器) 命令:
 *   0x20 MOTOR_CMD    : [duty_l_hi][duty_l_lo][duty_r_hi][duty_r_lo][flags]
 *                       - duty_l/r : int16, 左右轮带符号占空比(-10000~+10000)，
 *                                    正=前进 负=后退，绝对值=占空比(满量程10000)
 *                       - flags    : bit0=电机使能(enable)。为0时下板立即停车下电
 *
 * 二合一板 下板(执行器) -> 上板(大脑) 命令:
 *   0x21 ENC_FEEDBACK : [spd_l_hi][spd_l_lo][spd_r_hi][spd_r_lo][cnt_l_3..0][cnt_r_3..0]
 *                       - spd_l/r : int16, 左右轮编码器测速(每控制周期增量)
 *                       - cnt_l/r : int32, 左右轮编码器累计计数(大端)
 *
 * STM32 -> ESP32 命令:
 *   0x80 TELEMETRY    : [pos_hi][pos_lo][spd_l_hi][spd_l_lo][spd_r_hi][spd_r_lo][seg][batt_pct]
 *   0x81 ACK          : [echo_cmd]
 */

#define PROTO_HEADER        0xAA
#define PROTO_MAX_PAYLOAD   64

/* 命令定义 */
#define PROTO_CMD_LORA_SPEED   0x01
#define PROTO_CMD_LORA_STOP    0x02
#define PROTO_CMD_RADAR_DIST   0x03
#define PROTO_CMD_PING         0x10
#define PROTO_CMD_MOTOR_CMD    0x20
#define PROTO_CMD_ENC_FEEDBACK 0x21
#define PROTO_CMD_TELEMETRY    0x80
#define PROTO_CMD_ACK          0x81

/* MOTOR_CMD 帧 payload 长度与 flags 位定义 */
#define PROTO_MOTOR_CMD_LEN     5
#define PROTO_MOTOR_FLAG_ENABLE 0x01u

/* ENC_FEEDBACK 帧 payload 长度 */
#define PROTO_ENC_FEEDBACK_LEN  12

/* 解析状态机 */
typedef enum {
    PROTO_STATE_HEADER = 0,
    PROTO_STATE_LEN,
    PROTO_STATE_CMD,
    PROTO_STATE_PAYLOAD,
    PROTO_STATE_CRC
} ProtoState_t;

/* 接收到的完整帧 */
typedef struct {
    uint8_t cmd;
    uint8_t len;
    uint8_t payload[PROTO_MAX_PAYLOAD];
} ProtoFrame_t;

/* 回调函数类型 */
typedef void (*ProtoHandler_t)(const ProtoFrame_t *frame);

/**
 * @brief 初始化协议模块
 */
void Proto_Init(void);

/**
 * @brief 注册命令处理回调
 * @param cmd: 命令字节
 * @param handler: 处理函数
 */
void Proto_RegisterHandler(uint8_t cmd, ProtoHandler_t handler);

/**
 * @brief 从环形缓冲区取字节喂给解析器（主循环调用）
 *        每次调用处理所有可用字节
 */
void Proto_Process(void);

/**
 * @brief 发送帧（自动加 header/len/crc）
 * @param cmd: 命令字节
 * @param payload: 数据指针（可为 NULL 如果 len==0）
 * @param len: payload 长度
 */
void Proto_SendFrame(uint8_t cmd, const uint8_t *payload, uint8_t len);

/**
 * @brief 发送遥测数据到 ESP32
 */
void Proto_SendTelemetry(int16_t position, int16_t speed_l, int16_t speed_r,
                         uint8_t segment, uint8_t battery_pct);

/**
 * @brief 上板(大脑)发送电机指令帧到下板(执行器)
 * @param duty_l/duty_r: 左右轮带符号占空比(-10000~+10000)，正前进负后退
 * @param enable: 电机使能(1)/停车下电(0)
 */
void Proto_SendMotorCmd(int16_t duty_l, int16_t duty_r, uint8_t enable);

/**
 * @brief 下板(执行器)回传编码器反馈帧到上板(大脑)
 * @param speed_l/speed_r: 左右轮测速
 * @param cnt_l/cnt_r: 左右轮编码器累计计数
 */
void Proto_SendEncFeedback(int16_t speed_l, int16_t speed_r, int32_t cnt_l, int32_t cnt_r);

#endif /* __PROTOCOL_H__ */
