#include "sdi12_timer.h"

/**
 * @file sdi12_timer.cpp
 * @brief Timer1 driver for SDI-12 bit timing on ATmega1284P @ 14745600Hz
 *
 * Timer1 is a 16-bit hardware timer. With prescaler 1024 at 14745600Hz,
 * each tick is 69.4us. One SDI-12 bit at 1200 baud = 833us = ~12 ticks.
 *
 * TCCR1B prescaler bits for common values:
 *   CS10           = prescaler 1     → 0.068 us/tick
 *   CS11           = prescaler 8     → 0.543 us/tick
 *   CS11|CS10      = prescaler 64    → 4.34  us/tick
 *   CS12           = prescaler 256   → 17.4  us/tick
 *   CS12|CS10      = prescaler 1024  → 69.4  us/tick  ← used here
 * uRADMonitor - Global Environmental Monitoring Network , www.uradmonitor.com
 *
 * (C)2015  - 2026 MAGNASCI SRL , radu.motisan@magnasci.com
 */

void sdi12_timer_init(void) {
    TCCR1A = 0;                         // normal mode, no PWM output pins
    TCCR1B = (1 << CS12) | (1 << CS10); // prescaler 1024, timer starts running
    TCNT1  = 0;                         // clear counter
}

uint16_t sdi12_timer_now(void) {
    return TCNT1;
}
