#include "sdi12_lib.h"


#ifndef F_CPU
#error "F_CPU not defined!"
#endif
#if F_CPU != 14745600UL
#error "F_CPU wrong value!"
#endif
/**
 * @file sdi12.cpp
 * @brief SDI-12 slave primitives for AVR microcontrollers
 *
 * All pin access uses the SDI12_DDR / SDI12_PORT / SDI12_PIN / SDI12_BIT
 * macros defined in sdi12.h. Changing those macros (plus the PCINT macros)
 * is sufficient to move the implementation to a different GPIO pin.
 *
 * The ISR uses the following algorithm:
 * instead of sampling at fixed intervals, it fires on every pin transition
 * and counts how many bit-periods elapsed since the previous transition.
 * This approach requires no periodic timer ISR and tolerates slight jitter
 * in the data logger's timing.
 *
 * uRADMonitor - Global Environmental Monitoring Network , www.uradmonitor.com
 *
 * (C)2015  - 2026 MAGNASCI SRL , radu.motisan@magnasci.com
 */

// ── Internal state ────────────────────────────────────────────────────────────

/** @brief Ring buffer holding received bytes before the application reads them. */
static volatile uint8_t  rx_buf[SDI12_RX_BUF_SIZE];

/** @brief Index of the oldest unread byte in rx_buf. */
static volatile uint8_t  rx_head  = 0;

/** @brief Index where the next received byte will be written. */
static volatile uint8_t  rx_tail  = 0;

/**
 * @brief Current character assembly state.
 *
 * Counts how many data+parity bits have been received in the character
 * currently being assembled (0 = start bit just seen, 1-7 = data bits,
 * 8 = parity bit received). Value WAITING_FOR_START_BIT (0xFF) means
 * no character is in progress.
 */
static volatile uint8_t  rx_state = WAITING_FOR_START_BIT;

/**
 * @brief Bitmask with a '1' at the position of the next bit to fill.
 *
 * Starts at 0x01 (LSB) after the start bit and shifts left with each bit.
 * Used to set individual bits in rx_value without a separate bit counter.
 */
static volatile uint8_t  rx_mask  = 0;

/** @brief Byte being assembled by the ISR. Bits are filled LSB-first. */
static volatile uint8_t  rx_value = 0;

/**
 * @brief Timer1 value captured at the previous pin transition.
 *
 * The ISR subtracts this from the current TCNT1 to compute elapsed ticks.
 * 16-bit subtraction correctly handles timer rollover (two's complement wrap).
 */
static volatile uint16_t prev_tcnt = 0;

// ── RX buffer helpers ─────────────────────────────────────────────────────────

/**
 * @brief Push one byte into the tail of the ring buffer.
 *
 * Silently drops the byte if the buffer is full (tail+1 == head).
 * Called only from the ISR, so no locking is needed.
 *
 * @param c Byte to store.
 */
static void rx_buf_push(uint8_t c) {
    uint8_t next = (rx_tail + 1) % SDI12_RX_BUF_SIZE;
    if (next != rx_head) {  // drop silently if full
        rx_buf[rx_tail] = c;
        rx_tail = next;
    }
}

int16_t sdi12_rx_pop(void) {
    if (rx_head == rx_tail) return -1;  // empty
    uint8_t c = rx_buf[rx_head];
    rx_head = (rx_head + 1) % SDI12_RX_BUF_SIZE;
    return (int16_t)c;
}

uint8_t sdi12_rx_available(void) {
    return (rx_tail + SDI12_RX_BUF_SIZE - rx_head) % SDI12_RX_BUF_SIZE;
}

uint8_t sdi12_rx_find(uint8_t c) {
    uint8_t i = rx_head;
    while (i != rx_tail) {
        if (rx_buf[i] == c) return 1;
        i = (i + 1) % SDI12_RX_BUF_SIZE;
    }
    return 0;
}

// ── PCINT ISR ─────────────────────────────────────────────────────────────────
//
// Fires on every level change on SDI12_BIT (both rising and falling edges).
//
// Algorithm (EnviroDIY-style):
//   1. Record current timer value and pin level.
//   2. Compute elapsed ticks since last transition → divide by TICKS_PER_BIT
//      to get how many bit periods have passed.
//   3. If waiting for start bit: accept only a HIGH transition (Space = start bit).
//   4. Otherwise: determine how many bits belong to the current character
//      vs. a potential next character that may have already started.
//   5. Fill rx_value bits:
//      - Transition to HIGH means the preceding LOW bits were Mark=1.
//      - Transition to LOW means the current bit is Mark=1.
//      - HIGH=Space=0 (not set in rx_value), LOW=Mark=1 (set in rx_value).
//   6. When 8 bits (7 data + 1 parity) are complete: strip parity, push byte.
//
// SDI-12 character frame (LSB first, inverse logic):
//   [START=HIGH][b0][b1][b2][b3][b4][b5][b6][PARITY][STOP=LOW]
//    Space=0     <------------- 7 data bits ----------->        Mark=1

ISR(SDI12_PCINT_VECT) {
    uint16_t now     = sdi12_timer_now();
    uint8_t pin_high = (SDI12_PIN >> SDI12_BIT) & 1;  // 1=HIGH=Space=0, 0=LOW=Mark=1

    // elapsed ticks since last transition → number of bit periods
    uint16_t elapsed = now - prev_tcnt;
    uint8_t  rx_bits = (uint8_t)(elapsed / TICKS_PER_BIT);

    if (rx_state == WAITING_FOR_START_BIT) {
        // Only a HIGH (Space) transition can be a start bit.
        // LOW transitions during idle are just the line resting at Mark - ignore.
        if (!pin_high) { prev_tcnt = now; return; }

        // Start bit detected: begin assembling a new character
        rx_state = 0;       // 0 data+parity bits received so far
        rx_mask  = 0x01;    // next bit goes into bit position 0 (LSB)
        rx_value = 0x00;    // blank slate for incoming byte

    } else {
        // Mid-character: this transition is caused by a data, parity, or stop bit.

        if (rx_bits == 0) {
            // Glitch or bounce: less than one bit period has passed.
            // Update timestamp but otherwise ignore.
            prev_tcnt = now;
            return;
        }

        // Safety reset: if more than 12 bits have passed since the last transition,
        // we have missed part of a character (e.g. due to long Mark sequences or
        // inter-character gaps). Reset and wait for the next start bit.
        if (rx_bits > 12) {
            rx_state = WAITING_FOR_START_BIT;
            prev_tcnt = now;
            return;
        }

        // How many data+parity bits remain in the current character?
        // rx_state counts bits received so far (0-8). Total = 9 (7 data + parity + stop).
        uint8_t bits_left = 9 - rx_state;

        // Has a new character already started within this elapsed time?
        // If rx_bits > bits_left, some of those bits belong to the next character.
        uint8_t next_started = (rx_bits > bits_left) ? 1 : 0;

        // Bits that belong to the current character (capped at bits_left)
        uint8_t bits_this = next_started ? bits_left : rx_bits;

        rx_state += bits_this;

        if (pin_high) {
            // Transition to HIGH (Space=0): the preceding LOW bits were all Mark=1.
            // Fill in 'bits_this' ones using the mask, shifting left after each.
            while (bits_this-- > 0) {
                rx_value |= rx_mask;    // set this bit to 1 (Mark)
                rx_mask  <<= 1;         // advance to next bit position
            }
            // The current bit (now HIGH = Space = 0) is NOT set; just advance the mask.
            rx_mask <<= 1;

        } else {
            // Transition to LOW (Mark=1): bits before this were HIGH (Space=0, not set).
            // Only the current (last) bit is 1; advance mask past the zero bits first.
            rx_mask  <<= (bits_this - 1);   // skip over the preceding Space=0 bits
            rx_value |= rx_mask;            // set the current Mark=1 bit
        }

        // Have we received all 8 data+parity bits? (rx_state > 7 means bits 0-7 done)
        if (rx_state > 7) {
            rx_value &= 0x7F;       // strip bit 7 (parity) - keep only 7 data bits
            rx_buf_push(rx_value);  // deliver complete byte to application

            if (!pin_high || !next_started) {
                // Either we're in the stop bit (LOW) or no next character has started.
                // Return to idle and wait for the next start bit.
                rx_state = WAITING_FOR_START_BIT;
            } else {
                // The pin just went HIGH and a new character has already begun.
                // That HIGH transition IS the start bit of the next character.
                rx_state = 0;
                rx_mask  = 0x01;
                rx_value = 0x00;
            }
        }
    }

    prev_tcnt = now;  // save timestamp for next transition
}

// ── RX enable / disable / reset ───────────────────────────────────────────────

void sdi12_rx_enable(void) {
    rx_state  = WAITING_FOR_START_BIT;
    rx_head   = rx_tail = 0;
    prev_tcnt = sdi12_timer_now();
    PCICR  |=  (1 << SDI12_PCIE);      // enable pin-change interrupt group
    SDI12_PCMSK |= (1 << SDI12_PCINT_BIT); // enable this specific pin
}

void sdi12_rx_disable(void) {
    SDI12_PCMSK &= ~(1 << SDI12_PCINT_BIT); // mask this pin only; leave group enabled
}

void sdi12_rx_reset(void) {
    rx_head   = rx_tail = 0;
    rx_state  = WAITING_FOR_START_BIT;
    prev_tcnt = sdi12_timer_now();
}

// ── TX ────────────────────────────────────────────────────────────────────────

/**
 * @brief Transmit one bit. Inverse logic: mark=1 → LOW, space=0 → HIGH.
 *
 * @param mark 1 = Mark = LOW = binary 1 (idle/stop)
 *             0 = Space = HIGH = binary 0 (start/data 0)
 */
static void tx_bit(uint8_t mark) {
    if (mark)
        SDI12_PORT &= ~(1 << SDI12_BIT);   // Mark  = LOW
    else
        SDI12_PORT |=  (1 << SDI12_BIT);   // Space = HIGH
    _delay_us(833);                         // hold for exactly 1 bit @ 1200 baud
}

/**
 * @brief Transmit one byte in SDI-12 7E1 format (LSB first, even parity).
 *
 * Frame: START(Space=HIGH) + b0..b6 + PARITY(even) + STOP(Mark=LOW)
 *
 * @param b Byte to transmit (only bits 0-6 are sent; bit 7 is replaced by parity).
 */
static void tx_byte(uint8_t b) {
    uint8_t parity = 0;
    tx_bit(0);                              // start bit = Space = HIGH
    for (uint8_t i = 0; i < 7; i++) {
        uint8_t bit = (b >> i) & 1;
        parity ^= bit;                      // accumulate even parity
        tx_bit(bit);                        // data bits, LSB first
    }
    tx_bit(parity);                         // even parity bit
    tx_bit(1);                              // stop bit = Mark = LOW
}

void sdi12_tx_string(const char *s) {

    sdi12_rx_disable();                     // prevent echo reception during TX

    SDI12_DDR  |=  (1 << SDI12_BIT);       // switch pin to OUTPUT
    SDI12_PORT &= ~(1 << SDI12_BIT);       // drive LOW = Mark
    _delay_ms(8);                           // mandatory 8.33ms marking before response
                                            // (SDI-12 spec section 5.3)

    while (*s) tx_byte((uint8_t)*s++);     // transmit all bytes

    SDI12_PORT &= ~(1 << SDI12_BIT);       // return to Mark (LOW) after last byte
    _delay_ms(1);                           // brief hold before releasing bus
    SDI12_DDR  &= ~(1 << SDI12_BIT);       // switch pin back to INPUT

    sdi12_rx_enable();                      // re-enable RX for next command
}

// ── BREAK detection ───────────────────────────────────────────────────────────

uint8_t sdi12_detect_break(void) {
    // Line must be HIGH to be a BREAK (LOW = idle Mark = no BREAK)
    if (!((SDI12_PIN >> SDI12_BIT) & 1)) return 0;

    // Count loop iterations while the line stays HIGH.
    // At 14745600Hz with this loop structure, ~5000 counts ≈ 12ms.
    // Intentionally uses polling (not interrupts) to prevent the PCINT ISR
    // from misinterpreting the BREAK pulse as character data.
    uint16_t cnt = 0;
    while ((SDI12_PIN >> SDI12_BIT) & 1) cnt++;

    // Threshold: 5000 counts ≈ 12ms (minimum BREAK duration per SDI-12 spec)
    return (cnt >= 5000) ? 1 : 0;
}
