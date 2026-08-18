# Three-CAN integration notes

## Logical buses

```cpp
enum CanBusId : uint8_t {
    CAN_BUS_PRIMARY = 0,
    CAN_BUS_SECONDARY = 1,
    CAN_BUS_REMOTE = 2,
    CAN_BUS_COUNT = 3,
};
```

`can0` and `can1` are local to the LILYGO. `can2` is represented by a
`RemoteCanDriver` backed by the full-duplex UART connection to the Waveshare.

## UART protocol

Every packet is 22 bytes:

```text
magic[2], version, type, sequence[2], CAN ID[4], DLC, flags,
payload[8], CRC16[2]
```

Packet types cover RX frames, heartbeat, TX request/result, and mode control.
The sequence counter detects transport gaps; CRC16 rejects damaged packets.

## Scheduling requirements

- Service `can0` and `can1` before `can2`.
- Use a bounded RX budget per bus and loop iteration.
- Service the remote heartbeat before and after potentially slow Wi-Fi/HTTP
  work.
- UART writes must check `availableForWrite()` and drop/count a packet rather
  than blocking the control loop.
- Forward only the operational allow-list during normal operation. A deliberate
  CAN2 capture may temporarily request forward-all.

## Routing requirements

- Pass the source `CanBusId` into every frame dispatcher.
- A modified live frame returns only to its source bus.
- A generated frame uses an explicitly configured TX bus.
- The appearance of the same CAN ID on multiple buses does not authorize a
  state update or transmission on every bus.
- Remote frames remain visible to passive counters/logging, but enter a live
  state/control path only when that source or operation is assigned to CAN2.

## TX safety

The initial deployment should use `SATELLITE_FORCE_LISTEN_ONLY`:

- TWAI is installed in hardware Listen-Only mode;
- the TX queue length is zero;
- all UART TX requests are rejected;
- MODE packets cannot enable Normal mode.

If bidirectional CAN2 is enabled later, every send must pass a single
authorization point and match an explicitly configured CAN2 route.

## A/B isolation switch

The persistent `Ignore CAN2` switch stops and uninstalls TWAI, rejects TX and
MODE restart attempts, and keeps only the UART heartbeat alive. Disabling the
switch restarts the controller safely in Listen-Only mode.
