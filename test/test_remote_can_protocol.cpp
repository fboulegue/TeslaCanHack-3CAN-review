#include <cstdint>
#include <cstdio>
#include <cstring>

#include "../esp32/.firmware/remote_can_protocol.h"

static unsigned passed = 0;
static unsigned failed = 0;

#define CHECK(condition, message)                                              \
    do {                                                                       \
        if (condition) passed++;                                               \
        else { failed++; std::fprintf(stderr, "FAIL: %s (line %d)\n",       \
                                      message, __LINE__); }                    \
    } while (0)

int main() {
    uint8_t data[8] = {0x10, 0x21, 0x32, 0x43, 0x54, 0x65, 0x76, 0x87};
    uint8_t encoded[REMOTE_CAN_PACKET_SIZE] = {};
    remote_can_encode(encoded, REMOTE_CAN_TYPE_TX_REQUEST, 0x1234u, 0x3C2u, 8u,
                      REMOTE_CAN_FLAG_EXTENDED, data);

    RemoteCanPacket decoded = {};
    CHECK(remote_can_decode(encoded, &decoded), "valid packet decodes");
    CHECK(decoded.type == REMOTE_CAN_TYPE_TX_REQUEST, "TX request type preserved");
    CHECK(decoded.sequence == 0x1234u, "sequence little-endian");
    CHECK(decoded.can_id == 0x3C2u, "CAN id little-endian");
    CHECK(decoded.dlc == 8u, "DLC preserved");
    CHECK(decoded.flags == REMOTE_CAN_FLAG_EXTENDED, "flags preserved");
    CHECK(std::memcmp(decoded.data, data, sizeof(data)) == 0, "payload preserved");

    encoded[15] ^= 0x01u;
    CHECK(!remote_can_decode(encoded, &decoded), "CRC detects payload corruption");
    encoded[15] ^= 0x01u;

    for (uint8_t type = REMOTE_CAN_TYPE_RX_FRAME; type <= REMOTE_CAN_TYPE_MODE; type++) {
        remote_can_encode(encoded, type, type, 0u, 8u, 0u, data);
        CHECK(remote_can_decode(encoded, &decoded), "all protocol v2 types decode");
        CHECK(decoded.type == type, "protocol v2 type preserved");
    }

    encoded[3] = 0xFFu;
    remote_can_put_u16(encoded + 20, remote_can_crc16(encoded + 2, 18u));
    CHECK(!remote_can_decode(encoded, &decoded), "unknown packet type rejected");

    remote_can_encode(encoded, REMOTE_CAN_TYPE_HEARTBEAT, 1u, 0u, 8u, 0u, data);
    encoded[10] = 9u;
    remote_can_put_u16(encoded + 20, remote_can_crc16(encoded + 2, 18u));
    CHECK(!remote_can_decode(encoded, &decoded), "DLC above eight rejected");

    std::printf("Remote CAN protocol tests: %u passed, %u failed\n", passed, failed);
    return failed == 0 ? 0 : 1;
}
