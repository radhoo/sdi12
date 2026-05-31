#pragma once
#include <avr/io.h>
#include <stdint.h>

/**
 * @file sdi12_timer.h
 * @brief Timer1 driver for SDI-12 bit timing on ATmega1284P @ 14745600Hz
 *
 * Timer1 is configured in free-running normal mode with prescaler 1024.
 * It is never reset, allowing the ISR to measure elapsed time between
 * pin transitions using simple subtraction (correctly handles 16-bit rollover).
 *
 * Timing parameters:
 *   Clock:         14745600 Hz
 *   Prescaler:     1024
 *   Tick duration: 1024 / 14745600 = 69.4 us
 *   Rollover:      65536 ticks = ~4.55 seconds (safe for SDI-12 timeouts)
 *
 * SDI-12 baud rate: 1200 baud → 1 bit = 833 us
 *   Theoretical ticks/bit: 14745600 / 1024 / 1200 = 12
 *   Empirical ticks/bit:   11 (accounts for ISR entry/exit overhead)
 *
 * Recalibration procedure (if decoding fails on different hardware):
 *   1. Enable SDI12_DEBUG in sdi12_main.cpp
 *   2. Send any command to the sensor
 *   3. Observe "# CMD=..." output - if garbled, adjust TICKS_PER_BIT
 *   4. Try values: 10, 11, 12, 13
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

/**
 * @brief Number of Timer1 ticks per SDI-12 bit period.
 *
 * Empirically calibrated value. The theoretical value is:
 *   F_CPU / SDI12_TIMER_PRESCALER / SDI12_BAUD = 14745600 / 1024 / 1200 = 12
 * In practice, 11 works better due to ISR overhead on ATmega1284P.
 */
#define TICKS_PER_BIT  12

/**
 * @brief Timer1 prescaler value used for bit timing.
 * Changing this requires updating TICKS_PER_BIT accordingly.
 * Formula: TICKS_PER_BIT = F_CPU / SDI12_TIMER_PRESCALER / 1200
 */
#define SDI12_TIMER_PRESCALER  1024

/**
 * @brief Initialize Timer1 in free-running normal mode with prescaler 1024.
 *
 * Sets TCCR1A = 0 (normal mode, no PWM output), TCCR1B with CS12|CS10
 * (prescaler 1024), and clears TCNT1. Timer runs continuously and is
 * never reset by the SDI-12 driver.
 *
 * Must be called once before sdi12_rx_enable() and sei().
 */
void sdi12_timer_init(void);

/**
 * @brief Return the current Timer1 counter value.
 *
 * Used by the PCINT ISR to timestamp each pin transition.
 * Rolls over every ~4.55 seconds - safe because SDI-12 characters
 * are 10 bits at 1200 baud = 8.33ms, well within rollover period.
 *
 * @return Current 16-bit TCNT1 value.
 */
uint16_t sdi12_timer_now(void);
