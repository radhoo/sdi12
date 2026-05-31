#include "uradmonitor.h"
#if CONNECTIVITY == CONN_SDI12

#include "../devices/external/sdi12/sdi12_lib.h"
#include "../devices/external/sdi12/sdi12_timer.h"

void connectAP(bool init) {}

/**
 * @file sdi12_main.cpp
 * @brief SDI-12 slave application layer for uRADMonitor CITY
 *
 * Implements the SDI-12 command set on top of the sdi12.h primitives.
 * All SDI-12 protocol logic (command parsing, response formatting) lives here.
 * Hardware-specific details (pin, timer, ISR) are isolated in sdi12.h/sdi12.cpp.
 *
 * Supported commands:
 *   a!     Acknowledge Active       - confirm sensor presence
 *   aI!    Send Identification      - return manufacturer/model/serial info
 *   aM!    Start Measurement        - signal data is ready, report value count
 *   aDn!   Send Data (n = 0..5)     - return measured values for register n
 *   ?!     Address Query (broadcast)- respond to bus scan
 *
 * Data registers (aDn! responses):
 *   D0: uptime(s), warmup(s), watchdog counter, auto-reboot interval
 *   D1: temperature(C), humidity(%), pressure(Pa)
 *   D2: PM1, PM2.5, PM10 (ug/m3)
 *   D3: gas sensor 1 name + concentration, gas sensor 2 name + concentration
 *   D4: gas sensor 3 name + concentration, gas sensor 4 name + concentration
 *   D5: GPS latitude, longitude, altitude(m), speed(km/h), satellites, battery(V)
 *
 * SDI-12 response length constraints (spec: max 75 data chars per aDn! response):
 *   D0: ~30 chars  D1: ~25 chars  D2: ~15 chars
 *   D3: ~35 chars  D4: ~35 chars  D5: ~50 chars  → all within limit
 *
 *   * The ISR uses the following algorithm:
 * instead of sampling at fixed intervals, it fires on every pin transition
 * and counts how many bit-periods elapsed since the previous transition.
 * This approach requires no periodic timer ISR and tolerates slight jitter
 * in the data logger's timing.
 *
 * uRADMonitor - Global Environmental Monitoring Network , www.uradmonitor.com
 *
 * (C)2015  - 2026 MAGNASCI SRL , radu.motisan@magnasci.com
 */

// Uncomment to enable debug output on the serial console.
// With debug active, sendSerial() calls add enough delay that even marginal
// timing configurations work. Disable for production.
#define SDI12_DEBUG

// ── Build data response ───────────────────────────────────────────────────────

/**
 * @brief Format a aDn! data response into resp[].
 *
 * Prepends SDI12_ADDRESS and appends CR/LF. Each value is prefixed with
 * '+' (positive) or '-' (negative) per SDI-12 spec section 4.4.
 *
 * @param resp Output buffer. Must be at least 80 bytes.
 * @param reg  Register index (0-5). Unrecognised values produce address+CR/LF only.
 */
static void build_response(char *resp, uint8_t reg) {
    char tmp[100];
    resp[0] = SDI12_ADDRESS; resp[1] = '\0';
#if DEV_MODEL == MODEL_CITY
    if (reg == 0) {
        // D0: system status - uptime, warmup period, watchdog counter, auto-reboot (~30 chars)
        snprintf(tmp,sizeof(tmp),"+%lu", devices.time.getTotalSec());     strcat(resp,tmp);
        snprintf(tmp,sizeof(tmp),"+%u",  devices.eeprom.getWarmup());     strcat(resp,tmp);
        snprintf(tmp,sizeof(tmp),"+%lu", devices.wd.getCounter());        strcat(resp,tmp);
        snprintf(tmp,sizeof(tmp),"+%lu", devices.eeprom.getAutoReboot()); strcat(resp,tmp);

    } else if (reg == 1) {
        // D1: environmental - temperature (C), relative humidity (%), pressure (Pa) (~25 chars)
        snprintf(tmp,sizeof(tmp),"+%.2f",(double)devices.data.getTemperature()); strcat(resp,tmp);
        snprintf(tmp,sizeof(tmp),"+%.2f",(double)devices.data.getHumidity());    strcat(resp,tmp);
        snprintf(tmp,sizeof(tmp),"+%lu", devices.data.getPressure());            strcat(resp,tmp);

    } else if (reg == 2) {
        // D2: particulate matter - PM1, PM2.5, PM10 (ug/m3) (~15 chars)
        snprintf(tmp,sizeof(tmp),"+%u", devices.data.getPM1());  strcat(resp,tmp);
        snprintf(tmp,sizeof(tmp),"+%u", devices.data.getPM25()); strcat(resp,tmp);
        snprintf(tmp,sizeof(tmp),"+%u", devices.data.getPM10()); strcat(resp,tmp);

    } else if (reg == 3) {
        // D3: gas sensors 1 and 2 - name (string) + concentration (ppm/ppb) (~35 chars)
        snprintf(tmp,sizeof(tmp),"+%s",   devices.data.getGasName(0));          strcat(resp,tmp);
        snprintf(tmp,sizeof(tmp),"+%.3f",(double)devices.data.getGasCon(0));    strcat(resp,tmp);
        snprintf(tmp,sizeof(tmp),"+%s",   devices.data.getGasName(1));          strcat(resp,tmp);
        snprintf(tmp,sizeof(tmp),"+%.3f",(double)devices.data.getGasCon(1));    strcat(resp,tmp);

    } else if (reg == 4) {
        // D4: gas sensors 3 and 4 - name (string) + concentration (ppm/ppb) (~35 chars)
        snprintf(tmp,sizeof(tmp),"+%s",   devices.data.getGasName(2));          strcat(resp,tmp);
        snprintf(tmp,sizeof(tmp),"+%.3f",(double)devices.data.getGasCon(2));    strcat(resp,tmp);
        snprintf(tmp,sizeof(tmp),"+%s",   devices.data.getGasName(3));          strcat(resp,tmp);
        snprintf(tmp,sizeof(tmp),"+%.3f",(double)devices.data.getGasCon(3));    strcat(resp,tmp);

    } else if (reg == 5) {
        // D5: GPS and power - latitude, longitude, altitude(m), speed(km/h),
        //     satellite count, battery voltage(V) (~50 chars)
        snprintf(tmp,sizeof(tmp),"+%.6f",(double)devices.nmea.getLatitude());   strcat(resp,tmp);
        snprintf(tmp,sizeof(tmp),"+%.6f",(double)devices.nmea.getLongitude());  strcat(resp,tmp);
        snprintf(tmp,sizeof(tmp),"+%.2f",(double)devices.nmea.getAltitude());   strcat(resp,tmp);
        snprintf(tmp,sizeof(tmp),"+%.2f",(double)devices.nmea.getSpeed());      strcat(resp,tmp);
        snprintf(tmp,sizeof(tmp),"+%u",  devices.nmea.getSatellites());         strcat(resp,tmp);
        snprintf(tmp,sizeof(tmp),"+%.2f",(double)devices.data.battery);         strcat(resp,tmp);
    }
#endif
    strcat(resp, "\r\n");
}

// ── Process command ───────────────────────────────────────────────────────────

/**
 * @brief Parse and execute one SDI-12 command string.
 *
 * Commands must end with '!' and start with our address or '?'.
 * All responses are sent via sdi12_tx_string() which handles timing and
 * the mandatory 8.33ms marking prefix required by the SDI-12 spec.
 *
 * @param cmd Null-terminated command string, e.g. "0!" or "0D1!".
 */
static void process_command(const char *cmd) {

    uint8_t len = strlen(cmd);

    // command must have at least 2 characters: address + '!'
    if (len < 2) return;

    char addr = cmd[0];

    // accept our address (SDI12_ADDRESS) or '?' (broadcast to all sensors on bus)
    if (addr != SDI12_ADDRESS && addr != '?') return;

    // response buffer - 80 bytes is sufficient for any valid SDI-12 response
    // SDI-12 spec: max 75 data chars + 1 address + 2 CR/LF = 78 chars total
    char resp[80];

    // ── a! - Acknowledge Active ───────────────────────────────────────────────
    // Sent by the data logger to verify a sensor is present at address 'a'.
    // Response: address + CR/LF  (e.g. "0\r\n")
    if (len == 2 && cmd[1] == '!') {
    	char ack[4] = {SDI12_ADDRESS, '\r', '\n', '\0'};
        sdi12_tx_string(ack);
    }

    // ── aI! - Send Identification ─────────────────────────────────────────────
    // Data logger requests sensor identification. Response format (35 chars fixed):
    //   a        address             1 char
    //   CC       SDI-12 version      2 chars   "13" = v1.3
    //   MMMMMMMM manufacturer        8 chars   "uRADMoni"
    //   VVVVVV   model               6 chars   "trCITY"
    //   SSSSSSSS serial number       8 chars   hex device ID (e.g. "1C000075")
    // Example response: "013uRADMonitrCITY1C000075\r\n"
    else if (len == 3 && cmd[1] == 'I' && cmd[2] == '!') {
        snprintf(resp, sizeof(resp), "%c13uRADMonitrCITY%08lX\r\n",
                 SDI12_ADDRESS, devices.getDeviceID());
        sdi12_tx_string(resp);
    }

    // ── aM! - Start Measurement ───────────────────────────────────────────────
    // Data logger requests a measurement. Since uRADMonitor samples continuously,
    // data is always available immediately (ttt=000).
    // Response 1: atttn + CR/LF  where ttt=000, n=total value count
    //   a   = address
    //   000 = seconds until data ready (0 = immediately)
    //   24  = total values available across D0..D5
    // Response 2: after ttt seconds, sensor sends a\r\n to confirm data ready.
    //   Here we send it after a 50ms delay since data is already available.
    else if (len == 3 && cmd[1] == 'M' && cmd[2] == '!') {
        char r1[10] = {SDI12_ADDRESS,'0','0','0','2','4','\r','\n','\0'};
        sdi12_tx_string(r1);    // "000" seconds wait, 24 values available
        _delay_ms(50);
        char done[4] = {SDI12_ADDRESS, '\r', '\n', '\0'};
        sdi12_tx_string(done);  // data-ready notification
    }

    // ── aDn! - Send Data ──────────────────────────────────────────────────────
    // Data logger requests values from register n (0-5).
    // Response: address + values prefixed with '+'/'-' + CR/LF
    // Each register stays within the 75-char SDI-12 data limit.
    else if (len == 4 && cmd[1] == 'D' && cmd[3] == '!') {
        uint8_t reg = cmd[2] - '0';
        if (reg > 5) return;    // only registers 0-5 are defined
        build_response(resp, reg);
        sdi12_tx_string(resp);
    }

    // ── ?! - Address Query (broadcast) ───────────────────────────────────────
    // Data logger scans the bus without knowing sensor addresses.
    // All sensors respond with their address + CR/LF.
    // On a multi-sensor bus, sensors should randomise their response delay
    // to avoid collisions; here we respond immediately (single-sensor use).
    else if (addr == '?' && len == 2 && cmd[1] == '!') {
        char ack[4] = {SDI12_ADDRESS, '\r', '\n', '\0'};
        sdi12_tx_string(ack);
    }
}

// ── SDI-12 poll ───────────────────────────────────────────────────────────────

/**
 * @brief Check for BREAK and, if found, receive and process one command.
 *
 * Call this from the main loop on every iteration. The function returns
 * immediately if no BREAK is detected, so it does not block normal operation.
 *
 * Sequence:
 *   1. Poll for BREAK (HIGH pulse >= 12ms) - returns immediately if not found.
 *   2. Reset RX state machine to discard any noise captured during BREAK.
 *   3. Wait up to 100ms for '!' end-of-command marker to appear in RX buffer.
 *   4. Read complete command from buffer.
 *   5. Wait 15ms (SDI-12 spec requires >= 8.33ms before sensor responds).
 *   6. Dispatch to process_command().
 */
static void sdi12_poll(void) {
    if (!sdi12_detect_break()) return;

    // reset RX after BREAK - discard any noise captured during the pulse
    cli();
    sdi12_rx_reset();
    sei();

#ifdef SDI12_DEBUG
    sendSerial(buffer, BUFFER_SIZE, 0, 0, PSTR("# BREAK!\r\n"));
#endif

    // wait for '!' end-of-command marker, timeout after 100ms
    uint32_t timeout = 100000UL;
    while (timeout--) {
        cli();
        uint8_t found = sdi12_rx_find('!');
        sei();
        if (found) break;
        _delay_us(1);
    }

    // read command bytes from buffer until '!' or buffer empty
    char cmd[16] = {0};
    uint8_t ci = 0;
    while (ci < 15) {
        cli();
        int16_t b = sdi12_rx_pop();
        sei();
        if (b < 0) break;
        cmd[ci++] = (char)b;
        if (cmd[ci-1] == '!') break;
    }
    cmd[ci] = '\0';

#ifdef SDI12_DEBUG
    sendSerial(buffer, BUFFER_SIZE, 0, 0, PSTR("# CMD=%s\r\n"), cmd);
#endif

    if (ci >= 2) {
        // SDI-12 spec: sensor must wait at least 8.33ms before responding.
        // 15ms provides extra margin for strict data loggers.
        _delay_ms(15);
        process_command(cmd);
    }
}

// ── Main ──────────────────────────────────────────────────────────────────────

int main(void) {
    _delay_ms(500);
    devices.init();
    devices.start();

    // configure SDI-12 pin as input, no pull-up
    SDI12_DDR  &= ~(1 << SDI12_BIT);
    SDI12_PORT &= ~(1 << SDI12_BIT);

    sdi12_timer_init();     // start Timer1 for bit timing
    sdi12_rx_enable();      // arm PCINT on SDI-12 pin
    sei();                  // enable global interrupts

#ifdef SDI12_DEBUG
    sendSerial(buffer, BUFFER_SIZE, 0, 0,
        PSTR("# SDI-12 ready addr=%c TICKS=%u\r\n"),
        SDI12_ADDRESS, (uint16_t)TICKS_PER_BIT);
#endif

    while (1) {
        devices.loop();
        sdi12_poll();

        // On SDI-12 firmware we do not transmit data packets over the network,
        // but we still need to pet the watchdog and reset measurement buffers
        // every minute so sensor readings stay fresh.
        if (devices.cmdSend && devices.cmdRead == 1) {
            devices.beforeSend();
            devices.wd.petTheDog();
            devices.pcktsTotal++;
            devices.resetData();
            devices.cmdSend = 0;
        }
    }

    return 0;
}

#endif
