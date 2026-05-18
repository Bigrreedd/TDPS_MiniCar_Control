#include "Protocol.h"
#include "Uart_Config.h"

/* ========== 内部状态 ========== */
static ProtoState_t  s_state;
static uint8_t       s_len;        /* 期望 payload 长度 */
static uint8_t       s_cmd;
static uint8_t       s_payload_idx;
static uint8_t       s_payload_buf[PROTO_MAX_PAYLOAD];
static uint8_t       s_crc;        /* 累积 CRC */

/* 命令回调表（最多 8 条，够用） */
#define MAX_HANDLERS 8
static struct {
    uint8_t       cmd;
    ProtoHandler_t handler;
} s_handlers[MAX_HANDLERS];
static uint8_t s_handler_count = 0;

/* ========== 内部辅助 ========== */
static void Proto_FeedByte(uint8_t byte)
{
    switch (s_state) {

    case PROTO_STATE_HEADER:
        if (byte == PROTO_HEADER) {
            s_crc = 0;
            s_state = PROTO_STATE_LEN;
        }
        break;

    case PROTO_STATE_LEN:
        if (byte > PROTO_MAX_PAYLOAD) {
            /* 非法长度，回到初始态 */
            s_state = PROTO_STATE_HEADER;
        } else {
            s_len = byte;
            s_crc ^= byte;
            s_state = PROTO_STATE_CMD;
        }
        break;

    case PROTO_STATE_CMD:
        s_cmd = byte;
        s_crc ^= byte;
        s_payload_idx = 0;
        if (s_len == 0) {
            s_state = PROTO_STATE_CRC;
        } else {
            s_state = PROTO_STATE_PAYLOAD;
        }
        break;

    case PROTO_STATE_PAYLOAD:
        s_payload_buf[s_payload_idx++] = byte;
        s_crc ^= byte;
        if (s_payload_idx >= s_len) {
            s_state = PROTO_STATE_CRC;
        }
        break;

    case PROTO_STATE_CRC:
        if (byte == s_crc) {
            /* CRC 校验通过，分发到回调 */
            static ProtoFrame_t frame;
            frame.cmd = s_cmd;
            frame.len = s_len;
            for (uint8_t i = 0; i < s_len; i++) {
                frame.payload[i] = s_payload_buf[i];
            }
            for (uint8_t i = 0; i < s_handler_count; i++) {
                if (s_handlers[i].cmd == s_cmd && s_handlers[i].handler != 0) {
                    s_handlers[i].handler(&frame);
                    break;
                }
            }
            /* 发送 ACK（PING 不回 ACK 避免风暴） */
            if (s_cmd != PROTO_CMD_PING) {
                Proto_SendFrame(PROTO_CMD_ACK, &s_cmd, 1);
            }
        }
        /* 无论 CRC 是否通过都回到初始态 */
        s_state = PROTO_STATE_HEADER;
        break;

    default:
        s_state = PROTO_STATE_HEADER;
        break;
    }
}

/* ========== 公共接口 ========== */
void Proto_Init(void)
{
    s_state = PROTO_STATE_HEADER;
    s_handler_count = 0;
}

void Proto_RegisterHandler(uint8_t cmd, ProtoHandler_t handler)
{
    if (s_handler_count >= MAX_HANDLERS) return;
    s_handlers[s_handler_count].cmd = cmd;
    s_handlers[s_handler_count].handler = handler;
    s_handler_count++;
}

void Proto_Process(void)
{
    while (Uart2_BytesAvailable() > 0) {
        uint8_t byte = Uart2_ReadByteBlocking();
        Proto_FeedByte(byte);
    }
}

void Proto_SendFrame(uint8_t cmd, const uint8_t *payload, uint8_t len)
{
    if (len > PROTO_MAX_PAYLOAD) len = PROTO_MAX_PAYLOAD;

    uint8_t crc = len ^ cmd;
    Uart2_SendByte(PROTO_HEADER);
    Uart2_SendByte(len);
    Uart2_SendByte(cmd);
    for (uint8_t i = 0; i < len; i++) {
        Uart2_SendByte(payload[i]);
        crc ^= payload[i];
    }
    Uart2_SendByte(crc);
}

void Proto_SendTelemetry(int16_t position, int16_t speed_l, int16_t speed_r,
                         uint8_t segment, uint8_t battery_pct)
{
    uint8_t buf[8];
    buf[0] = (uint8_t)(position >> 8);
    buf[1] = (uint8_t)(position);
    buf[2] = (uint8_t)(speed_l >> 8);
    buf[3] = (uint8_t)(speed_l);
    buf[4] = (uint8_t)(speed_r >> 8);
    buf[5] = (uint8_t)(speed_r);
    buf[6] = segment;
    buf[7] = battery_pct;
    Proto_SendFrame(PROTO_CMD_TELEMETRY, buf, 8);
}
