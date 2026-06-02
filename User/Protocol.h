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
 * 二合一板 上板(传感) -> 下板(电机) 命令:
 *   0x20 SENSOR_DATA  : [pos_hi][pos_lo][flags][seg][gz_hi][gz_lo]
 *                       - pos     : int16, 循迹位置 = precise_position * 10（范围约 0~60）
 *                       - flags   : bit0=循线找到(found), bit1=启动(racing)
 *                       - seg     : 预留(0)，段状态由下板本地 Path 计算
 *                       - gz      : int16, 角速度 gz * 1000 (rad/s)，供下板角度环使用
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
#define PROTO_CMD_SENSOR_DATA  0x20
#define PROTO_CMD_TELEMETRY    0x80
#define PROTO_CMD_ACK          0x81

/* SENSOR_DATA 帧 payload 长度与 flags 位定义 */
#define PROTO_SENSOR_DATA_LEN  6
#define PROTO_SENSOR_FLAG_FOUND   0x01u
#define PROTO_SENSOR_FLAG_RACING  0x02u

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
 * @brief 上板发送传感数据帧到下板（二合一板）
 * @param position: 循迹位置 = precise_position * 10
 * @param found:    循线是否找到（1/0）
 * @param racing:   是否处于启动/运行状态（1/0）
 * @param gz_rads:  角速度 gz (rad/s)，内部按 *1000 编码为 int16
 */
void Proto_SendSensorData(int16_t position, uint8_t found, uint8_t racing, float gz_rads);

#endif /* __PROTOCOL_H__ */
