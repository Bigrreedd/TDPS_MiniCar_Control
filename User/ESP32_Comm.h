#ifndef __ESP32_COMM_H__
#define __ESP32_COMM_H__

#include "stm32f10x.h"
#include <stdint.h>

/* ============================================================================
 * ESP32S3-1 通信协议 v1.0
 * 硬件：STM32 UART ↔ ESP32S3-1 (GPIO17/18)
 * 波特率：115200 8N1
 * 帧格式：A5 5A | ver | type | seq(LE) | len(LE) | payload | checksum
 * ============================================================================ */

// 协议常量
#define ESP32_FRAME_HEADER_0    0xA5
#define ESP32_FRAME_HEADER_1    0x5A
#define ESP32_PROTOCOL_VERSION  0x01

// 帧类型定义
#define ESP32_TYPE_REQ_DECISION   0x01  // STM32 → ESP32: 请求障碍物决策
#define ESP32_TYPE_REQ_RADAR      0x02  // STM32 → ESP32: 请求雷达原始数据
#define ESP32_TYPE_AT_POSITION    0x03  // STM32 → ESP32: 通知到达检测位置 + 请求决策（推荐）
#define ESP32_TYPE_PING           0x04  // STM32 → ESP32: 心跳检测
#define ESP32_TYPE_LOG            0x07  // STM32 → ESP32: 日志/参数透传(ESP32 原样转 BLE A003，无 ACK)

#define ESP32_TYPE_DECISION       0x11  // ESP32 → STM32: 障碍物通行方向
#define ESP32_TYPE_RADAR          0x12  // ESP32 → STM32: 雷达数据
#define ESP32_TYPE_STATUS         0x20  // ESP32 → STM32: 状态心跳（预留）
#define ESP32_TYPE_ARCH_PASSED    0x30  // ESP32 → STM32: 拱门到达通知(06-06 Q1 定版:ESP 侧旧号
                                        //   0x12 RSP_ARCH_STATE 与我方 RADAR 冲突，双方统一 0x30)

// 单帧 payload 上限（队友协议规定 247；TX 超长自动拆帧，RX 受 ESP32_RX_BUF_SIZE 约束）
#define ESP32_MAX_PAYLOAD         247

// 通行决策
typedef enum {
    ESP32_GO_LEFT = 0,    // 从左侧通过（右侧有障碍）
    ESP32_GO_RIGHT = 1,   // 从右侧通过（左侧有障碍）
    ESP32_UNKNOWN = 2     // 无法判断
} ESP32_Decision_t;

// DECISION payload (2 字节)
typedef struct {
    uint8_t decision;         // 0=LEFT, 1=RIGHT, 2=UNKNOWN
    uint8_t radar_presence;   // 0=无, 1=检测到目标
} ESP32_DecisionPayload_t;

// RADAR payload (21 字节)
typedef struct {
    uint32_t timestamp_ms;    // 硬件时间戳 ms (LE)
    uint8_t presence;         // 0=无人, 1=检测到目标
    uint16_t distance;        // 目标距离值 (LE)
    uint16_t energy[8];       // 前 8 个距离门能量值 (每门 2B LE，覆盖 0-5.6m)
} ESP32_RadarPayload_t;

// 帧头结构（固定 9 字节 + payload）
typedef struct {
    uint8_t header[2];        // A5 5A
    uint8_t version;          // 0x01
    uint8_t type;             // 帧类型
    uint16_t seq;             // 序列号 (LE)
    uint16_t payload_len;     // payload 长度 (LE)
} ESP32_FrameHeader_t;

// 接收状态机
typedef enum {
    ESP32_RX_WAIT_HEADER_0,
    ESP32_RX_WAIT_HEADER_1,
    ESP32_RX_WAIT_VERSION,
    ESP32_RX_WAIT_TYPE,
    ESP32_RX_WAIT_SEQ_L,
    ESP32_RX_WAIT_SEQ_H,
    ESP32_RX_WAIT_LEN_L,
    ESP32_RX_WAIT_LEN_H,
    ESP32_RX_WAIT_PAYLOAD,
    ESP32_RX_WAIT_CHECKSUM
} ESP32_RxState_t;

// 接收缓冲区
#define ESP32_RX_BUF_SIZE 64
typedef struct {
    ESP32_RxState_t state;
    uint8_t version;
    uint8_t type;
    uint16_t seq;
    uint16_t payload_len;
    uint8_t payload[ESP32_RX_BUF_SIZE];
    uint16_t payload_idx;
    uint8_t checksum_calc;    // 计算中的校验和
    uint8_t checksum_recv;    // 接收到的校验和
} ESP32_RxFrame_t;

/* ============================================================================
 * 公共 API
 * ============================================================================ */

/**
 * @brief 初始化 ESP32 通信模块
 * @note  调用前需要先初始化对应的 UART（等待用户确认是哪个 UART）
 */
void ESP32_Comm_Init(void);

/**
 * @brief 发送请求：通知到达检测位置 + 请求障碍物决策（推荐使用）
 * @return 发送的帧序列号
 * @note  ESP32 会回复 DECISION (type=0x11)
 */
uint16_t ESP32_SendAtPosition(void);

/**
 * @brief 发送请求：仅请求障碍物决策（不带位置通知）
 * @return 发送的帧序列号
 */
uint16_t ESP32_SendReqDecision(void);

/**
 * @brief 发送请求：请求雷达原始数据
 * @return 发送的帧序列号
 * @note  ESP32 会回复 RADAR (type=0x12, 21 字节)
 */
uint16_t ESP32_SendReqRadar(void);

/**
 * @brief 发送心跳检测
 * @return 发送的帧序列号
 */
uint16_t ESP32_SendPing(void);

/**
 * @brief 发送日志/参数透传帧 (type=0x07)
 * @param payload: 日志字节（建议 ASCII/UTF-8 文本，一行一帧）
 * @param len: 字节数；> ESP32_MAX_PAYLOAD 时自动拆成多帧
 * @note  ESP32 不回 ACK，发完即走；ESP32 把 payload 原样转发 BLE A003 → 小程序面板
 */
void ESP32_SendLog(const uint8_t *payload, uint16_t len);

/**
 * @brief 获取拱门到达通知 (type=0x30，ESP32 主动下发)
 * @param arch_id: 输出拱门编号（payload[0]；ESP 发空 payload 时为 0）
 * @return 1=收到过新通知, 0=无
 * @note  调用后清除 "新数据" 标志；语义=车正穿过拱门(S-mode 第二触发源/段标定锚点)
 */
uint8_t ESP32_GetArchPassed(uint8_t *arch_id);

/**
 * @brief UART 接收中断回调（在 USART_IRQHandler 中调用）
 * @param data: 接收到的单字节数据
 * @note  逐字节喂给状态机，自动解析完整帧
 */
void ESP32_OnByteReceived(uint8_t data);

/**
 * @brief 获取最新的障碍物决策结果
 * @param decision: 输出决策（GO_LEFT/GO_RIGHT/UNKNOWN）
 * @param radar_presence: 输出雷达检测结果（0=无, 1=有目标）
 * @return 1=有新决策, 0=无新数据
 * @note  调用后会清除 "新数据" 标志
 */
uint8_t ESP32_GetDecision(ESP32_Decision_t *decision, uint8_t *radar_presence);

/**
 * @brief 获取最新的雷达原始数据
 * @param radar: 输出雷达数据结构体指针
 * @return 1=有新数据, 0=无新数据
 * @note  调用后会清除 "新数据" 标志
 */
uint8_t ESP32_GetRadarData(ESP32_RadarPayload_t *radar);

/**
 * @brief 获取链路状态
 * @return 1=链路正常, 0=链路断开（500ms 无响应）
 */
uint8_t ESP32_IsLinkAlive(void);

/**
 * @brief 定时器 tick（在 2ms 主控制周期中调用）
 * @note  用于超时检测
 */
void ESP32_Tick(void);

/* ============================================================================
 * UART 适配层接口（待用户指定具体 UART 后实现）
 * ============================================================================ */

/**
 * @brief 底层 UART 发送单字节（需用户根据实际 UART 实现）
 * @param data: 要发送的字节
 * @note  示例实现：
 *        void ESP32_UART_SendByte(uint8_t data) {
 *            while(USART_GetFlagStatus(USARTX, USART_FLAG_TXE) == RESET);
 *            USART_SendData(USARTX, data);
 *        }
 */
extern void ESP32_UART_SendByte(uint8_t data);

/**
 * @brief 底层 UART 初始化（需用户根据实际 UART 实现）
 * @note  配置：115200 8N1, 使能 RX 中断
 *        示例实现：
 *        void ESP32_UART_Init(void) {
 *            // GPIO + USART 初始化
 *            // 使能 USART_IT_RXNE 中断
 *            // 在中断中调用 ESP32_OnByteReceived()
 *        }
 */
extern void ESP32_UART_Init(void);

#endif /* __ESP32_COMM_H__ */
