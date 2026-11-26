# TeslaCanHack 3CAN Review Package

[![Build](https://github.com/fboulegue/TeslaCanHack-3CAN-review/actions/workflows/build.yml/badge.svg)](https://github.com/fboulegue/TeslaCanHack-3CAN-review/actions/workflows/build.yml)
[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](LICENSE)

Isolated, reusable **three-CAN transport**, **Waveshare ESP32-S3 satellite**, **RX-load isolation**, and **cross-bus routing** design extracted from the TeslaCanHack prototype.

It intentionally excludes vehicle-specific application handlers, release binaries, and unrelated prototype history so the architecture and hardware-safe Listen-Only satellite firmware can be reviewed and built cleanly.

---

## Hardware Architecture

- **Main Controller:** LILYGO T-2CAN (`can0` and `can1` local)
- **Remote CAN2 Satellite:** Waveshare Industrial ESP32-S3 RS485/CAN over full-duplex UART (`can2`)

```text
Waveshare SH1.0                  LILYGO T-2CAN
------------------------------------------------
GND                         ->  GND
IO1 / UART TX               ->  GPIO18 / UART RX
IO2 / UART RX               <-  GPIO17 / UART TX
3V3                         ->  not connected
```

- **CAN Bitrate:** `500 kbit/s`
- **UART Link:** `2,000,000 baud`, `8N1`
- **Waveshare CAN Pins:** TX `GPIO15` / RX `GPIO16`
- **Safety Default:** `waveshare-remote-satellite-listenonly` compiles with `SATELLITE_FORCE_LISTEN_ONLY` (TX permanently disabled in software and TWAI queue).

## Repository Contents

| Path | Description |
| --- | --- |
| `esp32/.firmware/remote_can_protocol.h` | Fixed 22-byte CRC16-protected UART frame & control protocol |
| `esp32/.firmware/remote_can_rx_filter.h` | Operational CAN ID allow-list preventing bus saturation |
| `esp32/.firmware/satellite/main.cpp` | Waveshare ESP32-S3 satellite firmware (Wi-Fi OTA, heartbeat, `Ignore CAN2` switch) |
| `esp32/reference/remote_can_driver.cpp` | LILYGO `RemoteCanDriver` integration excerpt |
| `esp32/reference/can_driver.h` | Three-bus driver interface contract |
| `docs/integration.md` | Routing, scheduling, and TX-safety design rules |
| `PROVENANCE.md` | Source commit provenance for the extracted changes |
| `test/` | Portable C++ unit tests for protocol encoding & RX filtering |

## Root Cause Isolated in Vehicle Testing

`CAN2` RX frames and counters arrived reliably over UART, but forwarding an unfiltered busy third bus caused intermittent `CAN0`/`CAN1` state and link oscillations (with `5–15` frames queued). Reproducing the issue in permanent Listen-Only mode proved it was not caused by remote TX, and disabling the Waveshare CAN controller while retaining the UART heartbeat restored stable `CAN0`/`CAN1` operation — isolating the issue to RX load, scheduling, and cross-bus dispatch.

## Build & Test

Run the portable host unit tests:

```bash
make -C test check
```

Build the permanently Listen-Only Waveshare satellite firmware:

```bash
pio run -d esp32 -e waveshare-remote-satellite-listenonly
```

> **Note:** The LILYGO driver files in `esp32/reference/` document the host-side contract and are not a standalone firmware target in this reduced repository.

## Safety & Disclaimer

This repository is provided for technical review and research purposes. Any hardware modifications or vehicle installations are at your own risk.

## License

GPL-3.0 — see [LICENSE](LICENSE).
