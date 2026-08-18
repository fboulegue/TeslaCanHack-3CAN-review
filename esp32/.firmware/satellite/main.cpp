#include <Arduino.h>
#include <driver/twai.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Update.h>
#include <Preferences.h>
#include "../remote_can_protocol.h"
#include "../remote_can_rx_filter.h"

#ifndef PIN_SAT_CAN_TX
#define PIN_SAT_CAN_TX 15
#endif
#ifndef PIN_SAT_CAN_RX
#define PIN_SAT_CAN_RX 16
#endif
#ifndef PIN_SAT_UART_TX
#define PIN_SAT_UART_TX 1
#endif
#ifndef PIN_SAT_UART_RX
#define PIN_SAT_UART_RX 2
#endif

#if defined(SATELLITE_FORCE_LISTEN_ONLY)
static constexpr bool SATELLITE_RX_ONLY = true;
#else
static constexpr bool SATELLITE_RX_ONLY = false;
#endif

static HardwareSerial g_link(1);
static WebServer g_web(80);
static uint8_t g_packet[REMOTE_CAN_PACKET_SIZE] = {};
static uint8_t g_packet_pos = 0;
static uint16_t g_tx_sequence = 0;
static uint32_t g_can_rx_count = 0;
static uint32_t g_can_rx_forwarded = 0;
static uint32_t g_can_rx_filtered = 0;
static uint32_t g_can_rx_dropped = 0;
static uint32_t g_can_tx_ok = 0;
static uint32_t g_can_tx_failed = 0;
static uint32_t g_last_main_packet_ms = 0;
static uint32_t g_last_heartbeat_ms = 0;
static bool g_listen_only = true;
static bool g_forward_all_rx = false;
static bool g_ignore_can2 = false;
static bool g_twai_started = false;
static bool g_twai_recovering = false;
static bool g_ota_in_progress = false;
static bool g_ota_success = false;
static bool g_ota_rejected = false;
static uint8_t g_ota_error = 0;
static size_t g_ota_written = 0;
static uint32_t g_reboot_at_ms = 0;

static const char SATELLITE_AP_SSID[] = "FSD-Satelit";
static const char SATELLITE_AP_PASSWORD[] = "12345678";

static const char SATELLITE_PAGE[] PROGMEM = R"SATPAGE(
<!doctype html><html lang="de"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>FSD Satellite</title><style>
:root{color-scheme:dark;font-family:system-ui,-apple-system,sans-serif;background:#0b1118;color:#eaf2fa}
*{box-sizing:border-box}body{margin:0;padding:18px}.wrap{max-width:680px;margin:auto}
h1{font-size:1.55rem;margin:4px 0 3px}.sub{color:#8fa5b8;margin:0 0 18px}
.card{background:#131d27;border:1px solid #263848;border-radius:14px;padding:16px;margin:12px 0}
.head{display:flex;justify-content:space-between;align-items:center;margin-bottom:12px}.head h2{font-size:1rem;margin:0}
.pill{padding:5px 9px;border-radius:999px;background:#3a2024;color:#ff9ba4;font-size:.78rem}
.pill.on{background:#173b2b;color:#75eca8}.pill.off{background:#46351e;color:#ffc86b}.grid{display:grid;grid-template-columns:repeat(2,1fr);gap:9px}
.stat{background:#0d151d;border-radius:10px;padding:11px}.value{font-size:1.35rem;font-weight:700}.label{font-size:.72rem;color:#8fa5b8;margin-top:3px}
.mode{font-weight:700}.warn{font-size:.8rem;color:#ffc86b;line-height:1.4}
.toggle{display:flex;gap:12px;align-items:flex-start;padding:12px;background:#0d151d;border-radius:10px;cursor:pointer}.toggle input{width:22px;height:22px;margin:1px 0 0;accent-color:#f0a43c}.toggle b{display:block}.toggle small{display:block;color:#8fa5b8;line-height:1.35;margin-top:4px}
input[type=file]{width:100%;padding:10px;background:#0d151d;border:1px solid #30485b;border-radius:9px;color:#dce9f4}
button{width:100%;margin-top:10px;padding:12px;border:0;border-radius:9px;background:#2782ef;color:white;font-weight:700;font-size:.92rem}
button:disabled{opacity:.45}progress{width:100%;height:10px;margin-top:12px}.msg{min-height:1.3em;margin-top:8px;font-size:.82rem;color:#8fa5b8}
footer{font-size:.7rem;color:#60778a;text-align:center;margin-top:16px}@media(min-width:600px){.grid{grid-template-columns:repeat(4,1fr)}}
</style></head><body><div class="wrap">
<h1>FSD Remote CAN2 Satellite</h1><p class="sub">Waveshare ESP32-S3 &middot; 500 kbit/s CAN &middot; UART v2</p>
<section class="card"><div class="head"><h2>Live-Status</h2><span id="link" class="pill">LilyGO offline</span></div>
<div class="grid"><div class="stat"><div id="rx" class="value">0</div><div class="label">CAN RX</div></div>
<div class="stat"><div id="fwd" class="value">0</div><div class="label">UART Forwarded</div></div>
<div class="stat"><div id="filter" class="value">0</div><div class="label">Operational gefiltert</div></div>
<div class="stat"><div id="drop" class="value">0</div><div class="label">Dropped</div></div>
<div class="stat"><div id="tx" class="value">0</div><div class="label">CAN TX</div></div>
<div class="stat"><div id="err" class="value">0</div><div class="label">TX Errors</div></div></div>
<p>CAN-Modus: <span id="mode" class="mode">Listen-Only</span><br>RX-Weiterleitung: <span id="rxPolicy" class="mode">Operational</span></p>
<p id="rxOnly" class="warn" hidden>RX-Only-Sicherheitsbuild: CAN-TX ist fest im Programm deaktiviert und kann vom LilyGO nicht aktiviert werden.</p>
<p id="detail" class="sub" style="margin:0">Lade Status ...</p></section>
<section class="card"><div class="head"><h2>CAN2-Verbindung</h2><span id="ignoreState" class="pill on">Aktiv</span></div>
<label class="toggle"><input id="ignoreCan" type="checkbox" onchange="setIgnore(this.checked)"><span><b>CAN2 vollständig ignorieren</b><small>Stoppt den CAN-Controller. Es werden keine CAN2-Frames gelesen, an den LilyGO weitergeleitet oder gesendet. Die UART-Verbindung bleibt für den Status aktiv.</small></span></label>
<div id="ignoreMsg" class="msg"></div></section>
<section class="card"><div class="head"><h2>OTA Firmware Update</h2></div>
<p id="otaHint" class="warn">Nur eine passende Waveshare-Satellite-OTA-Datei verwenden. Keine LilyGO-, Full- oder Merged-Datei. Während des Updates bleibt CAN sicher auf Listen-Only.</p>
<form id="ota"><input id="file" type="file" accept=".bin,application/octet-stream" required>
<button id="upload" type="submit">SATELLITE UPDATE STARTEN</button></form>
<progress id="progress" max="100" value="0" hidden></progress><div id="msg" class="msg"></div></section>
<footer id="build">FSD Satellite</footer></div><script>
const $=id=>document.getElementById(id);function num(v){return Number(v||0).toLocaleString('de-DE')}
async function poll(){try{const r=await fetch('/api/status',{cache:'no-store'}),d=await r.json();
$('rx').textContent=num(d.can_rx);$('fwd').textContent=num(d.can_forwarded);$('filter').textContent=num(d.can_filtered);$('drop').textContent=num(d.can_dropped+d.can_missed);$('tx').textContent=num(d.can_tx);$('err').textContent=num(d.can_tx_failed);
$('mode').textContent=d.force_listen_only?'Listen-Only (fest)':(d.listen_only?'Listen-Only':'Active');$('mode').style.color=d.listen_only?'#79bfff':'#75eca8';
$('rxPolicy').textContent=d.ignore_can2?'Aus (CAN2 ignoriert)':(d.forward_all?'Alle Frames (CAN2 Stream)':'Operational');$('rxPolicy').style.color=d.ignore_can2?'#ffc86b':(d.forward_all?'#ffc86b':'#75eca8');
$('ignoreCan').checked=!!d.ignore_can2;$('ignoreState').textContent=d.ignore_can2?'Ignoriert':'Aktiv';$('ignoreState').className='pill '+(d.ignore_can2?'off':'on');
$('rxOnly').hidden=!d.force_listen_only;
$('otaHint').innerHTML=d.force_listen_only?'Nur eine <b>Waveshare Listen-Only OTA-Datei</b> verwenden. Keine LilyGO-, Full- oder Merged-Datei.':'Nur eine <b>Waveshare Satellite OTA-Datei</b> verwenden. Keine LilyGO-, Full- oder Merged-Datei.';
$('link').textContent=d.link_alive?'LilyGO verbunden':'LilyGO offline';$('link').className='pill'+(d.link_alive?' on':'');
$('detail').textContent=(d.ignore_can2?'CAN2 vollständig deaktiviert':'CAN '+(d.can_started?'bereit':'nicht gestartet')+' · Queue '+d.can_queued)+' · UART-Linkalter '+(d.link_age_ms===null?'--':d.link_age_ms+' ms');
$('build').textContent='Build '+d.build;}catch(e){$('link').textContent='Status nicht erreichbar';$('link').className='pill'}}
async function setIgnore(v){const c=$('ignoreCan');c.disabled=true;$('ignoreMsg').textContent='Einstellung wird gespeichert ...';try{const r=await fetch('/api/ignore',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:'enabled='+(v?'1':'0')});if(!r.ok)throw new Error(await r.text());$('ignoreMsg').textContent=v?'CAN2 ist jetzt vollständig deaktiviert.':'CAN2 ist wieder aktiv (Listen-Only).';}catch(e){$('ignoreMsg').textContent='Änderung fehlgeschlagen: '+e.message;}c.disabled=false;poll()}
setInterval(poll,1000);poll();$('ota').addEventListener('submit',e=>{e.preventDefault();const f=$('file').files[0];if(!f)return;
if(/(?:full|merged)/i.test(f.name)){ $('msg').textContent='Abgelehnt: Bitte die OTA-Datei auswählen.';return }
const x=new XMLHttpRequest(),fd=new FormData();fd.append('firmware',f);$('upload').disabled=true;$('progress').hidden=false;$('msg').textContent='Upload läuft ...';
x.upload.onprogress=e=>{if(e.lengthComputable)$('progress').value=Math.round(e.loaded*100/e.total)};
x.onload=()=>{$('msg').textContent=x.responseText||('HTTP '+x.status);if(x.status<200||x.status>=300)$('upload').disabled=false};
x.onerror=()=>{$('msg').textContent='Upload-Verbindung fehlgeschlagen.';$('upload').disabled=false};x.open('POST','/update');x.send(fd)});
</script></body></html>
)SATPAGE";

static bool link_write(uint8_t type, uint16_t sequence, uint32_t can_id,
                       uint8_t dlc, uint8_t flags, const uint8_t *data) {
    uint8_t encoded[REMOTE_CAN_PACKET_SIZE];
    remote_can_encode(encoded, type, sequence, can_id, dlc, flags, data);
    // Never let a busy CAN2 stream block the only task that consumes LilyGO
    // keepalives. A full UART queue is reported as a dropped remote frame and
    // the next loop immediately services the return direction.
    if (g_link.availableForWrite() < (int)sizeof(encoded)) return false;
    return g_link.write(encoded, sizeof(encoded)) == sizeof(encoded);
}

static bool twai_start(bool listen_only) {
#if defined(SATELLITE_FORCE_LISTEN_ONLY)
    // This variant is physically RX-only from the TWAI driver's perspective.
    // No UART command is allowed to create a transmit-capable controller.
    listen_only = true;
#endif
    if (g_twai_started) {
        twai_stop();
        twai_driver_uninstall();
        g_twai_started = false;
    }
    // Safe software state until the requested controller mode is confirmed.
    g_listen_only = true;
    g_twai_recovering = false;
    twai_general_config_t general = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)PIN_SAT_CAN_TX, (gpio_num_t)PIN_SAT_CAN_RX,
        listen_only ? TWAI_MODE_LISTEN_ONLY : TWAI_MODE_NORMAL);
    general.rx_queue_len = 256;
    general.tx_queue_len = listen_only ? 0 : 32;
    general.alerts_enabled = TWAI_ALERT_RX_QUEUE_FULL | TWAI_ALERT_BUS_OFF |
                             TWAI_ALERT_TX_FAILED;
    twai_timing_config_t timing = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t filter = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    if (twai_driver_install(&general, &timing, &filter) != ESP_OK) return false;
    if (twai_start() != ESP_OK) {
        twai_driver_uninstall();
        return false;
    }
    g_twai_started = true;
    g_listen_only = listen_only;
    return true;
}

static void twai_disable() {
    if (!g_twai_started) {
        g_listen_only = true;
        return;
    }
    twai_stop();
    twai_driver_uninstall();
    g_twai_started = false;
    g_twai_recovering = false;
    g_listen_only = true;
}

static bool consume_byte(uint8_t value, RemoteCanPacket *decoded) {
    if (g_packet_pos == 0u) {
        if (value == REMOTE_CAN_MAGIC0) g_packet[g_packet_pos++] = value;
        return false;
    }
    if (g_packet_pos == 1u) {
        if (value == REMOTE_CAN_MAGIC1) g_packet[g_packet_pos++] = value;
        else if (value != REMOTE_CAN_MAGIC0) g_packet_pos = 0u;
        return false;
    }
    g_packet[g_packet_pos++] = value;
    if (g_packet_pos < REMOTE_CAN_PACKET_SIZE) return false;
    g_packet_pos = 0u;
    return remote_can_decode(g_packet, decoded);
}

static void send_tx_result(uint16_t request_sequence, uint8_t status) {
    uint8_t data[8] = {};
    data[0] = status;
    remote_can_put_u32(data + 1, g_can_tx_failed);
    remote_can_put_u16(data + 5, request_sequence);
    link_write(REMOTE_CAN_TYPE_TX_RESULT, g_tx_sequence++, 0u, 7u, 0u, data);
}

static void handle_tx_request(const RemoteCanPacket &packet) {
    if (g_ignore_can2) {
        g_can_tx_failed++;
        send_tx_result(packet.sequence, REMOTE_CAN_TX_LISTEN_ONLY);
        return;
    }
#if defined(SATELLITE_FORCE_LISTEN_ONLY)
    // Defense-in-depth: reject every request even if controller state were ever
    // corrupted. TWAI is also installed with a zero-length TX queue above.
    g_can_tx_failed++;
    send_tx_result(packet.sequence, REMOTE_CAN_TX_LISTEN_ONLY);
    return;
#endif
    if (g_listen_only || !g_twai_started) {
        g_can_tx_failed++;
        send_tx_result(packet.sequence, REMOTE_CAN_TX_LISTEN_ONLY);
        return;
    }
    if (packet.dlc > 8u || (packet.flags & ~(REMOTE_CAN_FLAG_EXTENDED | REMOTE_CAN_FLAG_RTR))) {
        g_can_tx_failed++;
        send_tx_result(packet.sequence, REMOTE_CAN_TX_INVALID);
        return;
    }
    twai_message_t message = {};
    message.identifier = packet.can_id;
    message.extd = (packet.flags & REMOTE_CAN_FLAG_EXTENDED) != 0u;
    message.rtr = (packet.flags & REMOTE_CAN_FLAG_RTR) != 0u;
    message.data_length_code = packet.dlc;
    memcpy(message.data, packet.data, packet.dlc);
    if (twai_transmit(&message, pdMS_TO_TICKS(5)) == ESP_OK) {
        g_can_tx_ok++;
        send_tx_result(packet.sequence, REMOTE_CAN_TX_OK);
    } else {
        g_can_tx_failed++;
        send_tx_result(packet.sequence, REMOTE_CAN_TX_FAILED);
    }
}

static void service_twai_health() {
    if (!g_twai_started) return;
    twai_status_info_t status = {};
    if (twai_get_status_info(&status) != ESP_OK) return;
    if (status.state == TWAI_STATE_BUS_OFF && !g_twai_recovering) {
        if (twai_initiate_recovery() == ESP_OK) {
            g_twai_recovering = true;
            g_can_tx_failed++;
        }
    } else if (g_twai_recovering && status.state == TWAI_STATE_STOPPED) {
        if (twai_start() == ESP_OK) g_twai_recovering = false;
    }
}

static void service_link() {
    while (g_link.available() > 0) {
        RemoteCanPacket packet = {};
        if (!consume_byte((uint8_t)g_link.read(), &packet)) continue;
        g_last_main_packet_ms = millis();
        if (packet.type == REMOTE_CAN_TYPE_MODE) {
            // data[1] is independent of TX mode: normal operation forwards a
            // compact allow-list, while a CAN2 stream asks for the full bus.
            g_forward_all_rx = packet.data[1] != 0u;
            if (g_ignore_can2) {
                // Keep the UART heartbeat alive, but never let MODE packets
                // restart CAN while the persistent ignore switch is active.
                twai_disable();
                continue;
            }
#if defined(SATELLITE_FORCE_LISTEN_ONLY)
            // MODE packets remain link keepalives, but Active is ignored.
            if (!g_listen_only) twai_start(true);
#else
            const bool requested_listen_only = packet.data[0] == 0u;
            // OTA always owns the safe state. Ignore a queued Active request
            // until the successfully updated satellite has rebooted.
            if (!g_ota_in_progress && requested_listen_only != g_listen_only)
                twai_start(requested_listen_only);
#endif
        } else if (packet.type == REMOTE_CAN_TYPE_TX_REQUEST) {
            handle_tx_request(packet);
        }
    }

    // Fail safe: a broken UART/main controller can never leave the satellite
    // able to ACK or transmit indefinitely.
    const uint32_t now = millis();
    if (!g_listen_only && (now - g_last_main_packet_ms) > 2000u) twai_start(true);
}

static void forward_can_rx() {
    if (!g_twai_started) return;
    twai_message_t message = {};
    // Small bounded bursts keep UART RX/keepalive and WiFi independent from a
    // saturated Vehicle CAN. The 256-entry hardware queue absorbs the gaps.
    uint16_t budget = 64;
    uint8_t serviced = 0;
    while (budget-- > 0u && twai_receive(&message, 0) == ESP_OK) {
        g_can_rx_count++;
        if (!g_forward_all_rx &&
            (message.extd || !remote_can_operational_id(message.identifier))) {
            g_can_rx_filtered++;
            continue;
        }
        uint8_t flags = 0u;
        if (message.extd) flags |= REMOTE_CAN_FLAG_EXTENDED;
        if (message.rtr) flags |= REMOTE_CAN_FLAG_RTR;
        if (!link_write(REMOTE_CAN_TYPE_RX_FRAME, g_tx_sequence++, message.identifier,
                        message.data_length_code, flags, message.data)) {
            g_can_rx_dropped++;
        } else {
            g_can_rx_forwarded++;
        }
        if ((++serviced & 0x0Fu) == 0u) service_link();
    }
}

static void send_heartbeat() {
    const uint32_t now = millis();
    if ((now - g_last_heartbeat_ms) < 500u) return;
    g_last_heartbeat_ms = now;
    uint8_t data[8] = {};
    remote_can_put_u32(data, g_can_rx_count);
    remote_can_put_u32(data + 4, g_can_rx_dropped);
    link_write(REMOTE_CAN_TYPE_HEARTBEAT, g_tx_sequence++, g_can_tx_failed, 8u,
               g_listen_only ? 0u : 1u, data);
}

static void send_no_cache(int code, const char *type, const String &body) {
    g_web.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate");
    g_web.sendHeader("Pragma", "no-cache");
    g_web.send(code, type, body);
}

static void web_root() {
    g_web.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate");
    g_web.send_P(200, "text/html; charset=utf-8", SATELLITE_PAGE);
}

static void web_status() {
    const uint32_t now = millis();
    const bool link_alive = g_last_main_packet_ms != 0u &&
                            (now - g_last_main_packet_ms) <= 2000u;
    twai_status_info_t can_status = {};
    const bool have_can_status = g_twai_started &&
                                 twai_get_status_info(&can_status) == ESP_OK;
    const uint32_t can_missed = have_can_status
        ? can_status.rx_missed_count + can_status.rx_overrun_count : 0u;
    const uint32_t can_queued = have_can_status ? can_status.msgs_to_rx : 0u;
    char link_age[24];
    if (g_last_main_packet_ms == 0u) {
        strcpy(link_age, "null");
    } else {
        snprintf(link_age, sizeof(link_age), "%lu",
                 (unsigned long)(now - g_last_main_packet_ms));
    }

    char json[768];
    snprintf(json, sizeof(json),
             "{\"can_rx\":%lu,\"can_forwarded\":%lu,\"can_filtered\":%lu,"
             "\"can_dropped\":%lu,\"can_missed\":%lu,"
             "\"can_queued\":%lu,\"can_tx\":%lu,\"can_tx_failed\":%lu,"
             "\"can_started\":%s,\"listen_only\":%s,\"force_listen_only\":%s,"
             "\"forward_all\":%s,\"ignore_can2\":%s,\"link_alive\":%s,"
             "\"link_age_ms\":%s,\"ota\":%s,\"build\":\"%s %s\"}",
             (unsigned long)g_can_rx_count,
             (unsigned long)g_can_rx_forwarded,
             (unsigned long)g_can_rx_filtered,
             (unsigned long)g_can_rx_dropped,
             (unsigned long)can_missed,
             (unsigned long)can_queued,
             (unsigned long)g_can_tx_ok,
             (unsigned long)g_can_tx_failed,
             g_twai_started ? "true" : "false",
             g_listen_only ? "true" : "false",
             SATELLITE_RX_ONLY ? "true" : "false",
             g_forward_all_rx ? "true" : "false",
             g_ignore_can2 ? "true" : "false",
             link_alive ? "true" : "false",
             link_age,
             g_ota_in_progress ? "true" : "false",
             __DATE__, __TIME__);
    send_no_cache(200, "application/json; charset=utf-8", json);
}

static void web_set_ignore() {
    if (!g_web.hasArg("enabled")) {
        send_no_cache(400, "text/plain; charset=utf-8", "Parameter enabled fehlt");
        return;
    }
    const String value = g_web.arg("enabled");
    const bool valid_true = value == "1" || value == "true" || value == "on";
    const bool valid_false = value == "0" || value == "false" || value == "off";
    if (!valid_true && !valid_false) {
        send_no_cache(400, "text/plain; charset=utf-8", "Ungueltiger enabled-Wert");
        return;
    }

    const bool enabled = valid_true;
    if (enabled != g_ignore_can2) {
        if (enabled) {
            g_ignore_can2 = true;
            twai_disable();
        } else {
            // Start safely in Listen-Only. A later LilyGO MODE keepalive may
            // request Active only in the explicitly TX-capable build.
            if (!twai_start(true)) {
                send_no_cache(500, "text/plain; charset=utf-8",
                              "CAN2 konnte nicht gestartet werden");
                return;
            }
            g_ignore_can2 = false;
        }
        Preferences preferences;
        if (preferences.begin("fsd-satellite", false)) {
            preferences.putBool("ignore_can2", g_ignore_can2);
            preferences.end();
        }
        if (Serial) Serial.printf("[SAT] CAN2 ignore: %s\n", g_ignore_can2 ? "ON" : "OFF");
    }
    send_no_cache(200, "application/json; charset=utf-8",
                  g_ignore_can2 ? "{\"ignore_can2\":true}" : "{\"ignore_can2\":false}");
}

static bool ota_filename_allowed(const String &filename) {
    String lower = filename;
    lower.toLowerCase();
    return lower.endsWith(".bin") && lower.indexOf("full") < 0 &&
           lower.indexOf("merged") < 0 && lower.indexOf("lilygo") < 0;
}

static void ota_upload() {
    HTTPUpload &upload = g_web.upload();
    if (upload.status == UPLOAD_FILE_START) {
        g_ota_success = false;
        g_ota_rejected = false;
        g_ota_error = 0;
        g_ota_written = 0;
        g_reboot_at_ms = 0;

        if (!ota_filename_allowed(upload.filename)) {
            g_ota_rejected = true;
            return;
        }

        // Never leave the vehicle-facing controller in Normal mode while its
        // application image is being replaced.
        if (!g_listen_only && !twai_start(true)) {
            g_ota_rejected = true;
            return;
        }
        if (!Update.begin(UPDATE_SIZE_UNKNOWN, U_FLASH)) {
            g_ota_error = (uint8_t)Update.getError();
            g_ota_rejected = true;
            return;
        }
        g_ota_in_progress = true;
        if (Serial) Serial.printf("[OTA] Satellite upload started: %s\n",
                                  upload.filename.c_str());
    } else if (upload.status == UPLOAD_FILE_WRITE) {
        if (g_ota_rejected || !g_ota_in_progress) return;
        // A normal ESP application image begins with 0xE9 and this target's
        // application has at least four segments. A merged full-flash image
        // starts with the three-segment bootloader, so reject it even if it was
        // renamed to hide "full"/"merged" in the filename.
        if (g_ota_written == 0u &&
            (upload.currentSize < 2u || upload.buf[0] != 0xE9u ||
             upload.buf[1] < 4u || upload.buf[1] > 16u)) {
            Update.abort();
            g_ota_rejected = true;
            g_ota_in_progress = false;
            return;
        }
        const size_t written = Update.write(upload.buf, upload.currentSize);
        g_ota_written += written;
        if (written != upload.currentSize) {
            g_ota_error = (uint8_t)Update.getError();
            Update.abort();
            g_ota_rejected = true;
            g_ota_in_progress = false;
        }
    } else if (upload.status == UPLOAD_FILE_END) {
        if (g_ota_rejected || !g_ota_in_progress) return;
        if (Update.end(true)) {
            g_ota_success = true;
            // Leave enough time for the HTTP response to reach the browser.
            g_reboot_at_ms = millis() + 1800u;
            if (Serial) Serial.printf("[OTA] Satellite update complete: %lu bytes; reboot scheduled\n",
                                      (unsigned long)g_ota_written);
        } else {
            g_ota_error = (uint8_t)Update.getError();
            g_ota_rejected = true;
            g_ota_in_progress = false;
        }
    } else if (upload.status == UPLOAD_FILE_ABORTED) {
        Update.abort();
        g_ota_rejected = true;
        g_ota_in_progress = false;
        if (Serial) Serial.println("[OTA] Satellite upload aborted");
    }
}

static void ota_complete() {
    if (g_ota_success) {
        send_no_cache(200, "text/plain; charset=utf-8",
                      "Update erfolgreich. Der Satellite startet jetzt automatisch neu ...");
        return;
    }
    char message[180];
    if (g_ota_rejected && g_ota_error == 0u) {
        snprintf(message, sizeof(message),
                 "Update abgelehnt. Bitte ausschließlich die Satellite-OTA-.bin verwenden.");
    } else {
        snprintf(message, sizeof(message), "Update fehlgeschlagen (Fehler %u). Kein Neustart.",
                 (unsigned)g_ota_error);
    }
    send_no_cache(400, "text/plain; charset=utf-8", message);
}

static void web_begin() {
    WiFi.mode(WIFI_AP);
    WiFi.setSleep(false);
    IPAddress ip(192, 168, 4, 1);
    IPAddress gateway(192, 168, 4, 1);
    IPAddress subnet(255, 255, 255, 0);
    WiFi.softAPConfig(ip, gateway, subnet);
    const bool ap_ok = WiFi.softAP(SATELLITE_AP_SSID, SATELLITE_AP_PASSWORD);

    g_web.on("/", HTTP_GET, web_root);
    g_web.on("/api/status", HTTP_GET, web_status);
    g_web.on("/api/ignore", HTTP_POST, web_set_ignore);
    g_web.on("/update", HTTP_POST, ota_complete, ota_upload);
    g_web.onNotFound([]() {
        send_no_cache(404, "text/plain; charset=utf-8", "Nicht gefunden\n");
    });
    g_web.begin();
    if (Serial) {
        Serial.printf("[WiFi] Satellite AP %s: \"%s\" IP: %s\n",
                      ap_ok ? "ready" : "FAILED", SATELLITE_AP_SSID,
                      WiFi.softAPIP().toString().c_str());
        Serial.println("[WiFi] Satellite dashboard: http://192.168.4.1");
    }
}

void setup() {
    Serial.begin(115200);
    Serial.setTxTimeoutMs(0);
    g_link.setRxBufferSize(4096u);
    g_link.setTxBufferSize(8192u);
    g_link.begin(REMOTE_CAN_UART_BAUD, SERIAL_8N1, PIN_SAT_UART_RX, PIN_SAT_UART_TX);
    Preferences preferences;
    if (preferences.begin("fsd-satellite", true)) {
        g_ignore_can2 = preferences.getBool("ignore_can2", false);
        preferences.end();
    }
    if (!g_ignore_can2) twai_start(true);
    g_last_main_packet_ms = 0u;
    if (Serial) {
        Serial.println(SATELLITE_RX_ONLY
            ? "[SAT] Waveshare remote CAN2 RX-only safety build; TX permanently disabled"
            : "[SAT] Waveshare remote CAN2 ready in Listen-Only");
        if (g_ignore_can2) Serial.println("[SAT] CAN2 controller disabled by persistent ignore setting");
    }
    web_begin();
}

void loop() {
    g_web.handleClient();
    if (g_reboot_at_ms != 0u &&
        (int32_t)(millis() - g_reboot_at_ms) >= 0) {
        if (Serial) Serial.println("[OTA] Rebooting satellite now");
        delay(40);
        ESP.restart();
    }
    if (g_ota_in_progress) {
        delay(1);
        return;
    }
    service_link();
    send_heartbeat();
    service_twai_health();
    forward_can_rx();
    service_link();
    g_web.handleClient();
    // A real scheduling point even with no browser connected. This makes the
    // UART link behaviour identical with the dashboard open or closed.
    delay(1);
}
