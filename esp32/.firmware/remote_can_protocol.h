#pragma once

#include <stdint.h>
#include <string.h>

// Fixed-size, CRC protected UART protocol between the LilyGO main controller
// and the Waveshare CAN satellite. UART is full duplex, so CAN RX forwarding
// and TX requests can run independently without sharing a half-duplex line.
#define REMOTE_CAN_MAGIC0       0xA5u
#define REMOTE_CAN_MAGIC1       0x5Au
#define REMOTE_CAN_VERSION      2u
#define REMOTE_CAN_PACKET_SIZE 22u
#define REMOTE_CAN_UART_BAUD    2000000u

enum RemoteCanPacketType : uint8_t {
    REMOTE_CAN_TYPE_RX_FRAME   = 1u, // satellite -> LilyGO: frame received on CAN2
    REMOTE_CAN_TYPE_HEARTBEAT  = 2u, // either direction: link/watchdog keepalive
    REMOTE_CAN_TYPE_TX_REQUEST = 3u, // LilyGO -> satellite: transmit on CAN2
    REMOTE_CAN_TYPE_TX_RESULT  = 4u, // satellite -> LilyGO: result for TX request
    REMOTE_CAN_TYPE_MODE       = 5u, // LilyGO -> satellite: data[0] 0=listen, 1=active
};

#define REMOTE_CAN_FLAG_EXTENDED 0x01u
#define REMOTE_CAN_FLAG_RTR      0x02u

enum RemoteCanTxStatus : uint8_t {
    REMOTE_CAN_TX_OK          = 0u,
    REMOTE_CAN_TX_LISTEN_ONLY = 1u,
    REMOTE_CAN_TX_INVALID     = 2u,
    REMOTE_CAN_TX_FAILED      = 3u,
};

typedef struct RemoteCanPacket {
    uint8_t type;
    uint16_t sequence;
    uint32_t can_id;
    uint8_t dlc;
    uint8_t flags;
    uint8_t data[8];
} RemoteCanPacket;

static inline uint16_t remote_can_get_u16(const uint8_t *p) {
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static inline uint32_t remote_can_get_u32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static inline void remote_can_put_u16(uint8_t *p, uint16_t value) {
    p[0] = (uint8_t)(value & 0xFFu);
    p[1] = (uint8_t)((value >> 8) & 0xFFu);
}

static inline void remote_can_put_u32(uint8_t *p, uint32_t value) {
    p[0] = (uint8_t)(value & 0xFFu);
    p[1] = (uint8_t)((value >> 8) & 0xFFu);
    p[2] = (uint8_t)((value >> 16) & 0xFFu);
    p[3] = (uint8_t)((value >> 24) & 0xFFu);
}

static inline uint16_t remote_can_crc16(const uint8_t *data, uint16_t len) {
    uint16_t crc = 0xFFFFu;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t bit = 0; bit < 8u; bit++) {
            crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u)
                                  : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

// Layout: magic[2], version, type, sequence[2], can_id[4], dlc, flags,
// payload[8], CRC16 over bytes 2..19.
static inline void remote_can_encode(uint8_t *out, uint8_t type,
                                     uint16_t sequence, uint32_t can_id,
                                     uint8_t dlc, uint8_t flags,
                                     const uint8_t *data) {
    memset(out, 0, REMOTE_CAN_PACKET_SIZE);
    out[0] = REMOTE_CAN_MAGIC0;
    out[1] = REMOTE_CAN_MAGIC1;
    out[2] = REMOTE_CAN_VERSION;
    out[3] = type;
    remote_can_put_u16(out + 4, sequence);
    remote_can_put_u32(out + 6, can_id);
    out[10] = dlc;
    out[11] = flags;
    if (data != 0) memcpy(out + 12, data, 8u);
    remote_can_put_u16(out + 20, remote_can_crc16(out + 2, 18u));
}

static inline bool remote_can_decode(const uint8_t *in, RemoteCanPacket *out) {
    if (in == 0 || out == 0 || in[0] != REMOTE_CAN_MAGIC0 ||
        in[1] != REMOTE_CAN_MAGIC1 || in[2] != REMOTE_CAN_VERSION ||
        in[10] > 8u ||
        remote_can_get_u16(in + 20) != remote_can_crc16(in + 2, 18u)) {
        return false;
    }
    const uint8_t type = in[3];
    if (type < REMOTE_CAN_TYPE_RX_FRAME || type > REMOTE_CAN_TYPE_MODE) return false;
    out->type = type;
    out->sequence = remote_can_get_u16(in + 4);
    out->can_id = remote_can_get_u32(in + 6);
    out->dlc = in[10];
    out->flags = in[11];
    memcpy(out->data, in + 12, 8u);
    return true;
}
