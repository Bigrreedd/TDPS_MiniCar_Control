#include "ESP32_Comm.h"
#include <string.h>

/* ============================================================================
 * 内部状态
 * ============================================================================ */

// 发送序列号（自增，用于检测丢帧）
static uint16_t g_esp32_tx_seq = 0;

// 接收帧状态机
static ESP32_RxFrame_t g_esp32_rx_frame;

// 最新的决策数据
static ESP32_DecisionPayload_t g_esp32_decision;
static uint8_t g_esp32_decision_ready = 0;

// 最新的雷达数据
static ESP32_RadarPayload_t g_esp32_radar;
static uint8_t g_esp32_radar_ready = 0;

// 链路超时检测（单位：2ms tick）
static uint16_t g_esp32_link_timeout = 0;
#define ESP32_LINK_TIMEOUT_TICKS  250  // 500ms @ 2ms/tick

/* ============================================================================
 * 内部辅助函数
 * ============================================================================ */

/**
 * @brief 计算 XOR 校验和（从 version 到 payload 末尾）
 */
static uint8_t ESP32_CalcChecksum(uint8_t version, uint8_t type, uint16_t seq,
                                  uint16_t len, const uint8_t *payload)
{
    uint8_t checksum = 0;
    checksum ^= version;
    checksum ^= type;
    checksum ^= (seq & 0xFF);
    checksum ^= (seq >> 8);
    checksum ^= (len & 0xFF);
    checksum ^= (len >> 8);
    for (uint16_t i = 0; i < len; i++) {
        checksum ^= payload[i];
    }
    return checksum;
}

/**
 * @brief 发送完整帧（无 payload）
 */
static void ESP32_SendFrame(uint8_t type)
{
    uint8_t checksum;

    // 帧头
    ESP32_UART_SendByte(ESP32_FRAME_HEADER_0);
    ESP32_UART_SendByte(ESP32_FRAME_HEADER_1);

    // 版本
    ESP32_UART_SendByte(ESP32_PROTOCOL_VERSION);

    // 类型
    ESP32_UART_SendByte(type);

    // 序列号（LE）
    ESP32_UART_SendByte(g_esp32_tx_seq & 0xFF);
    ESP32_UART_SendByte(g_esp32_tx_seq >> 8);

    // Payload 长度（LE，0 表示无 payload）
    ESP32_UART_SendByte(0x00);
    ESP32_UART_SendByte(0x00);

    // 校验和
    checksum = ESP32_CalcChecksum(ESP32_PROTOCOL_VERSION, type, g_esp32_tx_seq, 0, NULL);
    ESP32_UART_SendByte(checksum);

    // 序列号自增
    g_esp32_tx_seq++;
}

/**
 * @brief 解析 DECISION payload
 */
static void ESP32_ParseDecision(const uint8_t *payload, uint16_t len)
{
    if (len >= 2) {
        g_esp32_decision.decision = payload[0];
        g_esp32_decision.radar_presence = payload[1];
        g_esp32_decision_ready = 1;
        g_esp32_link_timeout = 0;  // 收到响应，重置超时
    }
}

/**
 * @brief 解析 RADAR payload
 */
static void ESP32_ParseRadar(const uint8_t *payload, uint16_t len)
{
    if (len >= 21) {
        // timestamp_ms (4B LE)
        g_esp32_radar.timestamp_ms = (uint32_t)payload[0]
                                   | ((uint32_t)payload[1] << 8)
                                   | ((uint32_t)payload[2] << 16)
                                   | ((uint32_t)payload[3] << 24);

        // presence (1B)
        g_esp32_radar.presence = payload[4];

        // distance (2B LE)
        g_esp32_radar.distance = (uint16_t)payload[5] | ((uint16_t)payload[6] << 8);

        // energy[8] (16B, 每门 2B LE)
        for (uint8_t i = 0; i < 8; i++) {
            g_esp32_radar.energy[i] = (uint16_t)payload[7 + i*2]
                                    | ((uint16_t)payload[8 + i*2] << 8);
        }

        g_esp32_radar_ready = 1;
        g_esp32_link_timeout = 0;  // 收到响应，重置超时
    }
}

/* ============================================================================
 * 公共 API 实现
 * ============================================================================ */

void ESP32_Comm_Init(void)
{
    // 初始化底层 UART（用户需根据实际 UART 实现 ESP32_UART_Init）
    ESP32_UART_Init();

    // 重置状态
    g_esp32_tx_seq = 0;
    memset(&g_esp32_rx_frame, 0, sizeof(g_esp32_rx_frame));
    g_esp32_rx_frame.state = ESP32_RX_WAIT_HEADER_0;

    g_esp32_decision_ready = 0;
    g_esp32_radar_ready = 0;
    g_esp32_link_timeout = 0;
}

uint16_t ESP32_SendAtPosition(void)
{
    uint16_t seq = g_esp32_tx_seq;
    ESP32_SendFrame(ESP32_TYPE_AT_POSITION);
    return seq;
}

uint16_t ESP32_SendReqDecision(void)
{
    uint16_t seq = g_esp32_tx_seq;
    ESP32_SendFrame(ESP32_TYPE_REQ_DECISION);
    return seq;
}

uint16_t ESP32_SendReqRadar(void)
{
    uint16_t seq = g_esp32_tx_seq;
    ESP32_SendFrame(ESP32_TYPE_REQ_RADAR);
    return seq;
}

uint16_t ESP32_SendPing(void)
{
    uint16_t seq = g_esp32_tx_seq;
    ESP32_SendFrame(ESP32_TYPE_PING);
    return seq;
}

void ESP32_OnByteReceived(uint8_t data)
{
    ESP32_RxFrame_t *frame = &g_esp32_rx_frame;

    switch (frame->state)
    {
    case ESP32_RX_WAIT_HEADER_0:
        if (data == ESP32_FRAME_HEADER_0) {
            frame->state = ESP32_RX_WAIT_HEADER_1;
            frame->checksum_calc = 0;
        }
        break;

    case ESP32_RX_WAIT_HEADER_1:
        if (data == ESP32_FRAME_HEADER_1) {
            frame->state = ESP32_RX_WAIT_VERSION;
        } else {
            frame->state = ESP32_RX_WAIT_HEADER_0;  // 重新同步
        }
        break;

    case ESP32_RX_WAIT_VERSION:
        frame->version = data;
        frame->checksum_calc ^= data;
        frame->state = ESP32_RX_WAIT_TYPE;
        break;

    case ESP32_RX_WAIT_TYPE:
        frame->type = data;
        frame->checksum_calc ^= data;
        frame->state = ESP32_RX_WAIT_SEQ_L;
        break;

    case ESP32_RX_WAIT_SEQ_L:
        frame->seq = data;
        frame->checksum_calc ^= data;
        frame->state = ESP32_RX_WAIT_SEQ_H;
        break;

    case ESP32_RX_WAIT_SEQ_H:
        frame->seq |= ((uint16_t)data << 8);
        frame->checksum_calc ^= data;
        frame->state = ESP32_RX_WAIT_LEN_L;
        break;

    case ESP32_RX_WAIT_LEN_L:
        frame->payload_len = data;
        frame->checksum_calc ^= data;
        frame->state = ESP32_RX_WAIT_LEN_H;
        break;

    case ESP32_RX_WAIT_LEN_H:
        frame->payload_len |= ((uint16_t)data << 8);
        frame->checksum_calc ^= data;
        frame->payload_idx = 0;

        if (frame->payload_len == 0) {
            // 无 payload，直接等校验和
            frame->state = ESP32_RX_WAIT_CHECKSUM;
        } else if (frame->payload_len > ESP32_RX_BUF_SIZE) {
            // Payload 过长，丢弃帧
            frame->state = ESP32_RX_WAIT_HEADER_0;
        } else {
            frame->state = ESP32_RX_WAIT_PAYLOAD;
        }
        break;

    case ESP32_RX_WAIT_PAYLOAD:
        frame->payload[frame->payload_idx++] = data;
        frame->checksum_calc ^= data;

        if (frame->payload_idx >= frame->payload_len) {
            frame->state = ESP32_RX_WAIT_CHECKSUM;
        }
        break;

    case ESP32_RX_WAIT_CHECKSUM:
        frame->checksum_recv = data;

        // 校验和验证
        if (frame->checksum_recv == frame->checksum_calc) {
            // 帧有效，根据类型解析
            switch (frame->type)
            {
            case ESP32_TYPE_DECISION:
                ESP32_ParseDecision(frame->payload, frame->payload_len);
                break;

            case ESP32_TYPE_RADAR:
                ESP32_ParseRadar(frame->payload, frame->payload_len);
                break;

            case ESP32_TYPE_STATUS:
                // 预留：状态心跳处理
                g_esp32_link_timeout = 0;
                break;

            default:
                // 未知类型，忽略
                break;
            }
        }

        // 重置状态机，等待下一帧
        frame->state = ESP32_RX_WAIT_HEADER_0;
        break;

    default:
        frame->state = ESP32_RX_WAIT_HEADER_0;
        break;
    }
}

uint8_t ESP32_GetDecision(ESP32_Decision_t *decision, uint8_t *radar_presence)
{
    if (g_esp32_decision_ready) {
        if (decision != NULL) {
            *decision = (ESP32_Decision_t)g_esp32_decision.decision;
        }
        if (radar_presence != NULL) {
            *radar_presence = g_esp32_decision.radar_presence;
        }
        g_esp32_decision_ready = 0;  // 清除标志
        return 1;
    }
    return 0;
}

uint8_t ESP32_GetRadarData(ESP32_RadarPayload_t *radar)
{
    if (g_esp32_radar_ready && radar != NULL) {
        memcpy(radar, &g_esp32_radar, sizeof(ESP32_RadarPayload_t));
        g_esp32_radar_ready = 0;  // 清除标志
        return 1;
    }
    return 0;
}

uint8_t ESP32_IsLinkAlive(void)
{
    return (g_esp32_link_timeout < ESP32_LINK_TIMEOUT_TICKS) ? 1 : 0;
}

void ESP32_Tick(void)
{
    // 每 2ms 调用一次，累加超时计数
    if (g_esp32_link_timeout < 0xFFFF) {
        g_esp32_link_timeout++;
    }
}

/* ============================================================================
 * 弱符号默认实现（用户需在自己的代码中重定义）
 * ============================================================================ */

__attribute__((weak)) void ESP32_UART_SendByte(uint8_t data)
{
    // 默认空实现，用户需根据实际 UART 重写此函数
    // 示例：
    // while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
    // USART_SendData(USART2, data);
    (void)data;
}

__attribute__((weak)) void ESP32_UART_Init(void)
{
    // 默认空实现，用户需根据实际 UART 重写此函数
    // 示例：
    // RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    // GPIO + USART 初始化，115200 8N1
    // USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    // 在中断中调用 ESP32_OnByteReceived(USART_ReceiveData(USART2));
}
