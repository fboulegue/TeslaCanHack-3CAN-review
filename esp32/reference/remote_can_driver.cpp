// Reference excerpt from the LilyGO CanDriver implementation.
// This file documents the integration point and is not built standalone.
// ── UART remote driver (bidirectional Waveshare CAN2 satellite) ──────────────
#if defined(CAN_DRIVER_REMOTE_UART)
class RemoteCanDriver : public CanDriver {
    HardwareSerial serial_;
    const char *label_;
    int rx_pin_;
    int tx_pin_;
    uint8_t packet_[REMOTE_CAN_PACKET_SIZE] = {};
    uint8_t packet_pos_ = 0;
    bool begun_ = false;
    bool listen_only_ = true;
    bool forward_all_rx_ = false;
    bool sequence_seen_ = false;
    uint16_t expected_sequence_ = 0;
    uint16_t tx_sequence_ = 0;
    uint32_t last_packet_ms_ = 0;
    uint32_t last_control_ms_ = 0;
    uint32_t rx_count_ = 0;
    uint32_t tx_count_ = 0;
    uint32_t framing_errors_ = 0;
    uint32_t sequence_gaps_ = 0;
    uint32_t uart_tx_errors_ = 0;
    uint32_t peer_tx_errors_ = 0;
    uint32_t peer_dropped_ = 0;

    bool write_packet(uint8_t type, uint32_t can_id, uint8_t dlc,
                      uint8_t flags, const uint8_t *data) {
        if (!begun_) return false;
        uint8_t encoded[REMOTE_CAN_PACKET_SIZE];
        remote_can_encode(encoded, type, tx_sequence_++, can_id, dlc, flags, data);
        // Keep the main loop non-blocking if the remote UART is temporarily
        // congested. MODE retries and feature TX attempts will be retried by
        // their normal cadence instead of starving CAN0/CAN1 and WiFi.
        if (serial_.availableForWrite() < (int)sizeof(encoded)) {
            uart_tx_errors_++;
            return false;
        }
        if (serial_.write(encoded, sizeof(encoded)) != sizeof(encoded)) {
            uart_tx_errors_++;
            return false;
        }
        return true;
    }

    bool send_mode() {
        uint8_t data[8] = {};
        data[0] = listen_only_ ? 0u : 1u;
        data[1] = forward_all_rx_ ? 1u : 0u;
        const bool ok = write_packet(REMOTE_CAN_TYPE_MODE, 0u, 2u, 0u, data);
        // Do not postpone the next retry when the UART write did not enter the
        // hardware buffer. This keepalive must not depend on a dashboard yield.
        if (ok) last_control_ms_ = millis();
        return ok;
    }

    bool consume_byte(uint8_t value, RemoteCanPacket *decoded) {
        if (packet_pos_ == 0u) {
            if (value == REMOTE_CAN_MAGIC0) packet_[packet_pos_++] = value;
            return false;
        }
        if (packet_pos_ == 1u) {
            if (value == REMOTE_CAN_MAGIC1) packet_[packet_pos_++] = value;
            else if (value != REMOTE_CAN_MAGIC0) packet_pos_ = 0u;
            return false;
        }
        packet_[packet_pos_++] = value;
        if (packet_pos_ < REMOTE_CAN_PACKET_SIZE) return false;
        packet_pos_ = 0u;
        if (!remote_can_decode(packet_, decoded)) {
            framing_errors_++;
            return false;
        }
        last_packet_ms_ = millis();
        if (sequence_seen_) {
            uint16_t gap = (uint16_t)(decoded->sequence - expected_sequence_);
            if (gap != 0u) {
                if (gap < 0x8000u) sequence_gaps_ += gap;
                else framing_errors_++;
            }
        }
        expected_sequence_ = (uint16_t)(decoded->sequence + 1u);
        sequence_seen_ = true;
        return true;
    }

    bool handle_control_packet(const RemoteCanPacket &decoded) {
        if (decoded.type == REMOTE_CAN_TYPE_HEARTBEAT) {
            peer_dropped_ = remote_can_get_u32(decoded.data + 4);
            peer_tx_errors_ = decoded.can_id;
            return true;
        }
        if (decoded.type == REMOTE_CAN_TYPE_TX_RESULT) {
            if (decoded.data[0] == REMOTE_CAN_TX_OK) tx_count_++;
            peer_tx_errors_ = remote_can_get_u32(decoded.data + 1);
            return true;
        }
        return decoded.type != REMOTE_CAN_TYPE_RX_FRAME;
    }

public:
    RemoteCanDriver(const char *label, int rx_pin, int tx_pin)
        : serial_(1), label_(label), rx_pin_(rx_pin), tx_pin_(tx_pin) {}

    bool begin(bool listen_only) override {
        listen_only_ = listen_only;
        if (!begun_) {
            serial_.setRxBufferSize(8192u);
            serial_.setTxBufferSize(8192u);
            serial_.begin(REMOTE_CAN_UART_BAUD, SERIAL_8N1, rx_pin_, tx_pin_);
            begun_ = true;
        }
        send_mode();
        Serial.printf("[CAN] %s Remote CAN UART RX+TX @ %lu baud (RX=%d TX=%d), %s requested\n",
                      label_, (unsigned long)REMOTE_CAN_UART_BAUD, rx_pin_, tx_pin_,
                      listen_only_ ? "Listen-Only" : "Active");
        return true;
    }

    bool send(const CanFrame &frame) override {
        if (!begun_ || listen_only_ || !linkAlive(millis()) || frame.dlc > 8u) return false;
        uint8_t flags = 0u;
        if (frame.ext) flags |= REMOTE_CAN_FLAG_EXTENDED;
        if (frame.req) flags |= REMOTE_CAN_FLAG_RTR;
        return write_packet(REMOTE_CAN_TYPE_TX_REQUEST, frame.id, frame.dlc,
                            flags, frame.data);
    }

    bool receive(CanFrame &frame) override {
        if (!begun_) return false;
        while (serial_.available() > 0) {
            RemoteCanPacket decoded = {};
            if (!consume_byte((uint8_t)serial_.read(), &decoded)) continue;
            if (handle_control_packet(decoded)) continue;
            frame.id = decoded.can_id;
            frame.ext = (decoded.flags & REMOTE_CAN_FLAG_EXTENDED) != 0u;
            frame.req = (decoded.flags & REMOTE_CAN_FLAG_RTR) != 0u;
            frame.dlc = decoded.dlc;
            memcpy(frame.data, decoded.data, 8u);
            rx_count_++;
            return true;
        }
        return false;
    }

    uint32_t errorCount() override {
        return framing_errors_ + sequence_gaps_ + uart_tx_errors_ + peer_tx_errors_;
    }
    uint32_t txCount() override { return tx_count_; }
    uint32_t rxCount() override { return rx_count_; }
    void setListenOnly(bool enable) override {
        if (listen_only_ == enable) return;
        listen_only_ = enable;
        send_mode();
    }
    bool hardwarePresent() override { return last_packet_ms_ != 0u; }
    bool linkAlive(uint32_t now_ms) override {
        return last_packet_ms_ != 0u && (now_ms - last_packet_ms_) <= 2000u;
    }
    uint32_t droppedCount() override { return peer_dropped_ + sequence_gaps_; }
    void setRemoteRxForwardAll(bool enable) override {
        if (forward_all_rx_ == enable) return;
        forward_all_rx_ = enable;
        // Send immediately so opening/closing a capture changes the satellite
        // policy without waiting for the periodic keepalive.
        send_mode();
    }
    void serviceHealth() override {
        const uint32_t now = millis();
        // Repeating MODE is both a keepalive and a way to recover when either
        // board boots later than the other. The satellite watchdog returns to
        // hardware Listen-Only if these messages stop.
        if ((now - last_control_ms_) >= 500u) send_mode();
    }
};
#endif
