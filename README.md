# TeslaCanHack 3CAN review package

This private repository contains only the reusable three-CAN transport,
Waveshare satellite, RX-load isolation, and routing design extracted from the
TeslaCanHack prototype. It intentionally excludes the rest of the private
firmware, vehicle-specific handlers, release binaries, and unrelated Git
history.

It is a review package for the proposed combination of:

- a LILYGO T-2CAN as the main controller for local `can0` and `can1`; and
- a Waveshare Industrial ESP32-S3 RS485/CAN as remote `can2` over UART.

The four requested commits are follow-up changes, not a standalone foundation.
The minimum UART protocol and satellite source they depend on are included so
the package can be understood and the Waveshare firmware can be built.

## Hardware and wiring

```text
Waveshare SH1.0                  LILYGO T-2CAN
------------------------------------------------
GND                         ->  GND
IO1 / UART TX               ->  GPIO18 / UART RX
IO2 / UART RX               <-  GPIO17 / UART TX
3V3                         ->  not connected
```

- CAN speed: 500 kbit/s
- UART: 2,000,000 baud, 8N1
- Waveshare CAN TX/RX GPIOs: 15/16
- The safe satellite target is permanently hardware Listen-Only.

## Repository contents

- `esp32/.firmware/remote_can_protocol.h`: fixed 22-byte CRC-protected UART
  protocol.
- `esp32/.firmware/remote_can_rx_filter.h`: normal-operation allow-list that
  prevents a busy third bus from flooding the LilyGO.
- `esp32/.firmware/satellite/main.cpp`: complete Waveshare firmware with Wi-Fi
  OTA, heartbeat/watchdog, permanent Listen-Only support, and persistent
  `Ignore CAN2` control.
- `esp32/reference/remote_can_driver.cpp`: the LilyGO `RemoteCanDriver`
  integration excerpt.
- `esp32/reference/can_driver.h`: the three-bus driver contract used by the
  prototype.
- `docs/integration.md`: routing, scheduling, and TX-safety requirements.
- `PROVENANCE.md`: exact source commits requested for review.

## What failed in the vehicle test

CAN2 RX itself worked. Real frames and counters arrived through the UART link.
The problem appeared once the busy third bus was forwarded: local CAN0/CAN1
state and transmissions became intermittent, the satellite link sometimes
oscillated online/offline, and the remote RX queue commonly held 5-15 frames.

The same behavior occurred with the satellite permanently Listen-Only, so the
problem was not caused by remote CAN transmission. Stopping the Waveshare CAN
controller while retaining the UART heartbeat restored stable CAN0/CAN1
operation. This isolated the problem to remote RX load, scheduling, and
cross-bus state dispatch.

## Build and test

Build the permanently Listen-Only Waveshare firmware:

```bash
cd esp32
pio run -e waveshare-remote-satellite-listenonly
```

Run the portable protocol/filter tests:

```bash
make -C test check
```

The LilyGO reference excerpt documents the driver contract but is deliberately
not a standalone firmware target in this reduced repository.

## License

GPL-3.0, matching the source project.
