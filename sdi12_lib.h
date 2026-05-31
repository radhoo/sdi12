#pragma once

#ifndef F_CPU
#define F_CPU 14745600UL
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>
#include "sdi12_timer.h"

/**
 * @file sdi12.h
 * @brief SDI-12 slave primitives for AVR microcontrollers
 *
 * Single-wire half-duplex implementation using one configurable GPIO pin.
 * Direct connection to SDI-12 bus, no external transistor required.
 *
 * ── Electrical levels ────────────────────────────────────────────────────────
 *   IDLE / MARK = 0V  = LOW  = binary 1  (line pulled LOW by data logger)
 *   BREAK       = 5V  = HIGH = binary 0  (data logger drives HIGH, min 12ms)
 *   SPACE       = 5V  = HIGH = binary 0
 *
 * ── RX implementation ────────────────────────────────────────────────────────
 *   Pin-change interrupt fires on every level transition.
 *   Timer1 timestamps each transition; elapsed ticks / TICKS_PER_BIT = bit count.
 *   Bit assembly follows the logic:
 *     - HIGH transition → previous LOW bits were Mark=1, fill them in
 *     - LOW transition  → current bit is Mark=1
 *     - Inverse logic: HIGH=Space=0, LOW=Mark=1
 *   Frame format: START(HIGH) + b0..b6 LSB-first + PARITY(even) + STOP(LOW)
 *
 * ── TX implementation ────────────────────────────────────────────────────────
 *   Bit-bang with _delay_us(833) per bit at 1200 baud.
 *   8.33ms marking (LOW) before first byte, as required by SDI-12 spec.
 *   Frame format: same 7E1 as RX.
 *
 * ── Hardware configuration ───────────────────────────────────────────────────
 *   All pin-specific macros are defined below and can be changed to use
 *   any GPIO pin on any AVR port. The PCINT vector, mask register, and
 *   enable bit must match the chosen pin (see AVR datasheet).
 *
 *   Example for ATmega1284P PD6:
 *     PD6 = PCINT30, vector PCINT3_vect, mask PCMSK3 bit 6, enable PCIE3
 *
 *   Example for ATmega1284P PC2:
 *     PC2 = PCINT18, vector PCINT1_vect, mask PCMSK1 bit 2, enable PCIE1
 *
 * This approach requires no periodic timer ISR and tolerates slight jitter
 * in the data logger's timing.
 * 
 * uRADMonitor - Global Environmental Monitoring Network , www.uradmonitor.com
 *
 * (C)2015  - 2026 MAGNASCI SRL , radu.motisan@magnasci.com
 */

// ── Pin configuration ─────────────────────────────────────────────────────────
//
// To use a different pin, change all six macros below to match.
// Consult the ATmega1284P datasheet section "Pin Change Interrupt" for
// the correct PCINT number, vector, mask register, and enable bit for each pin.
//
// ATmega1284P pin-change mapping reference:
//   Port A (PA0-PA7): PCINT0-7,   vector PCINT0_vect, mask PCMSK0, enable PCIE0
//   Port B (PB0-PB7): PCINT8-15,  vector PCINT1_vect, mask PCMSK1, enable PCIE1
//   Port C (PC0-PC7): PCINT16-23, vector PCINT2_vect, mask PCMSK2, enable PCIE2
//   Port D (PD0-PD7): PCINT24-31, vector PCINT3_vect, mask PCMSK3, enable PCIE3

/** @brief Data direction register for the SDI-12 pin (e.g. DDRD for PORTD) */
#define SDI12_DDR        DDRD

/** @brief Output register for the SDI-12 pin (e.g. PORTD) */
#define SDI12_PORT       PORTD

/** @brief Input register for the SDI-12 pin (e.g. PIND) */
#define SDI12_PIN        PIND

/** @brief Bit position of the SDI-12 pin within the port (e.g. PD6 = 6) */
#define SDI12_BIT        PD6

/**
 * @brief Pin-change interrupt vector for the chosen pin.
 *
 * Must match the port group containing SDI12_BIT:
 *   PORTA → PCINT0_vect
 *   PORTB → PCINT1_vect
 *   PORTC → PCINT2_vect
 *   PORTD → PCINT3_vect
 *
 * This macro is used to define the ISR:  ISR(SDI12_PCINT_VECT) { ... }
 */
#define SDI12_PCINT_VECT  PCINT3_vect

/**
 * @brief Pin-change interrupt enable bit in PCICR register.
 *
 * Must match the port group:
 *   PORTA → PCIE0
 *   PORTB → PCIE1
 *   PORTC → PCIE2
 *   PORTD → PCIE3
 */
#define SDI12_PCIE        PCIE3

/**
 * @brief Pin-change mask register for the chosen pin.
 *
 * Must match the port group:
 *   PORTA → PCMSK0
 *   PORTB → PCMSK1
 *   PORTC → PCMSK2
 *   PORTD → PCMSK3
 */
#define SDI12_PCMSK       PCMSK3

/**
 * @brief PCINT bit number within SDI12_PCMSK for the chosen pin.
 *
 * For ATmega1284P:
 *   PD0=PCINT24, PD1=PCINT25, PD2=PCINT26, PD3=PCINT27,
 *   PD4=PCINT28, PD5=PCINT29, PD6=PCINT30, PD7=PCINT31
 *
 * The bit position within PCMSK3 equals (PCINT_number - 24) for PORTD,
 * which equals the pin number itself (PD6 → bit 6 in PCMSK3).
 */
#define SDI12_PCINT_BIT   PCINT30

// ── SDI-12 address ────────────────────────────────────────────────────────────

/**
 * @brief SDI-12 sensor address (single ASCII character).
 *
 * Valid addresses per SDI-12 specification v1.3:
 *   '0'-'9' : 10 numeric addresses
 *   'A'-'Z' : 26 uppercase alphabetic addresses
 *   'a'-'z' : 26 lowercase alphabetic addresses
 *   Total   : 62 unique sensor addresses per bus
 *
 * Rules:
 *   - Each sensor on a shared bus must have a unique address.
 *   - Use '0' (default) when only one sensor is connected.
 *   - The data logger can reassign the address at runtime using command aAb!
 *     (changes sensor at address 'a' to address 'b').
 *   - Address '?' is reserved for broadcast queries and must not be used here.
 */
#define SDI12_ADDRESS  ((char)'0')

// ── Buffer configuration ──────────────────────────────────────────────────────

/**
 * @brief RX ring buffer size in bytes. Must be a power of 2.
 *
 * SDI-12 commands are short (max ~10 chars including address and '!'),
 * but the buffer must also hold any inter-character gaps and noise.
 * 32 bytes is sufficient for all standard SDI-12 commands.
 */
#define SDI12_RX_BUF_SIZE  32

// ── Internal state sentinel ───────────────────────────────────────────────────

/**
 * @brief Sentinel value for rx_state indicating the ISR is waiting for a start bit.
 *
 * The ISR uses rx_state to track how many data+parity bits have been received
 * in the current character (0-8). This value (0xFF) signals that no character
 * is in progress and the next HIGH transition should be treated as a start bit.
 */
#define WAITING_FOR_START_BIT  0xFF

// ── Public API ────────────────────────────────────────────────────────────────

/**
 * @brief Initialize and enable the SDI-12 RX interrupt on SDI12_BIT.
 *
 * Configures PCICR and SDI12_PCMSK to enable pin-change interrupts on
 * the configured pin. Resets the RX buffer, state machine, and timer
 * reference. Must be called after sdi12_timer_init() and before sei().
 *
 * Safe to call multiple times (e.g. after TX to re-enable RX).
 */
void sdi12_rx_enable(void);

/**
 * @brief Disable the SDI-12 RX interrupt on SDI12_BIT.
 *
 * Clears the pin's bit in SDI12_PCMSK. The PCICR enable bit (SDI12_PCIE)
 * is left set; only this specific pin's interrupt is masked.
 * Called automatically by sdi12_tx_string() before transmitting to prevent
 * the TX signal from being received back as RX data.
 */
void sdi12_rx_disable(void);

/**
 * @brief Reset the RX state machine and buffer without touching PCICR/PCMSK.
 *
 * Clears rx_head, rx_tail, sets rx_state = WAITING_FOR_START_BIT, and
 * updates prev_tcnt to the current timer value.
 *
 * Call this (with interrupts disabled) after BREAK detection to discard
 * any noise captured during the BREAK pulse and prepare for the command.
 *
 * Example:
 *   cli();
 *   sdi12_rx_reset();
 *   sei();
 */
void sdi12_rx_reset(void);

/**
 * @brief Return the number of bytes currently available in the RX buffer.
 *
 * This count may increase at any time if the ISR is active.
 * For a consistent snapshot, call with interrupts disabled.
 *
 * @return Number of bytes available (0 to SDI12_RX_BUF_SIZE-1).
 */
uint8_t sdi12_rx_available(void);

/**
 * @brief Remove and return one byte from the front of the RX buffer.
 *
 * Non-blocking. Call with interrupts disabled for safe access.
 *
 * @return The next byte as uint8_t cast to int16_t, or -1 if buffer is empty.
 */
int16_t sdi12_rx_pop(void);

/**
 * @brief Search the RX buffer for a specific byte without consuming it.
 *
 * Scans from head to tail without modifying the buffer.
 * Used to detect the '!' end-of-command marker before reading the full command.
 * Call with interrupts disabled for a consistent view of the buffer.
 *
 * @param c Byte value to search for.
 * @return 1 if found anywhere in the buffer, 0 otherwise.
 */
uint8_t sdi12_rx_find(uint8_t c);

/**
 * @brief Transmit a null-terminated string on the SDI-12 bus at 1200 baud 7E1.
 *
 * Sequence:
 *   1. Disable RX interrupt (sdi12_rx_disable)
 *   2. Switch SDI12_BIT to OUTPUT
 *   3. Drive LOW (Mark) for 8ms - mandatory marking before response (SDI-12 spec)
 *   4. Transmit each byte as: START(HIGH) + 7 data bits LSB-first + even parity + STOP(LOW)
 *      Each bit held for exactly 833us (_delay_us is cycle-accurate at known F_CPU)
 *   5. Drive LOW (Mark) for 1ms after last byte
 *   6. Switch SDI12_BIT back to INPUT
 *   7. Re-enable RX interrupt (sdi12_rx_enable)
 *
 * @param s Null-terminated ASCII string to transmit. Must not be NULL.
 */
void sdi12_tx_string(const char *s);

/**
 * @brief Detect a valid SDI-12 BREAK signal on SDI12_BIT.
 *
 * A BREAK is a HIGH pulse of at least 12ms sent by the data logger to
 * wake all sensors on the bus before issuing a command.
 *
 * Implementation: polls the pin in a tight loop, counting iterations.
 * A count >= 5000 corresponds to approximately 12ms at 14745600Hz.
 * This threshold is conservative; typical BREAK pulses are 15-30ms.
 *
 * Note: this function uses polling (not interrupts) deliberately, to avoid
 * the PCINT ISR corrupting rx_state during the BREAK pulse. The caller
 * should call sdi12_rx_reset() with interrupts disabled immediately after
 * this function returns 1.
 *
 * @return 1 if a valid BREAK was detected, 0 if the line was LOW or
 *         the HIGH pulse was shorter than the minimum BREAK duration.
 */
uint8_t sdi12_detect_break(void);
