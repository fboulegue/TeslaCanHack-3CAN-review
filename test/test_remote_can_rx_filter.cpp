#include <cstdio>
#include <cstdlib>

#include "../esp32/.firmware/remote_can_rx_filter.h"

static int checks = 0;

#define CHECK(cond, msg) do { \
    checks++; \
    if (!(cond)) { std::fprintf(stderr, "FAIL: %s\n", msg); std::exit(1); } \
} while (0)

int main() {
    CHECK(remote_can_operational_id(0x3FDu), "autopilot control is operational");
    CHECK(remote_can_operational_id(0x370u), "AntiNag is operational");
    CHECK(remote_can_operational_id(0x331u), "autopilot config is operational");
    CHECK(remote_can_operational_id(0x3C2u), "observed Juniper candidate remains visible");
    CHECK(remote_can_operational_id(0x102u), "door indicator remains visible");
    CHECK(!remote_can_operational_id(0x000u), "unrelated low id is filtered");
    CHECK(!remote_can_operational_id(0x555u), "unrelated vehicle id is filtered");
    CHECK(!remote_can_operational_id(0x18DAF110u), "extended diagnostic id is filtered");

    std::printf("remote CAN RX filter: %d checks passed\n", checks);
    return 0;
}
