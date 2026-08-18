# Requested change provenance

This review package is derived only from the reusable three-CAN portions of
the following requested commits:

1. `4c44c5b9367188e235801ac9287aa80331042ce3` — Routing and UART-link isolation
2. `d5ecdefe9ebfb00fbb030d907210ab28b10eea8d` — Permanent Listen-Only satellite
3. `e37c3fbc71c872890c6a41211e2c48bd7434d620` — CAN2 RX-load isolation
4. `8e4bde4f12cdf42efadfcc7738ec6a4fba562472` — Persistent Ignore CAN2 A/B switch

Those commits depend on the earlier remote-UART protocol and Waveshare
satellite foundation. Only the minimum required protocol, satellite, and
LilyGO driver-reference files are included here. Release binaries, unrelated
application changes, and the original repository history are excluded.
