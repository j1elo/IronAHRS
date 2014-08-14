/*
 * timer.h
 *
 * Created: 29/06/2012 17:43:04
 * Author: Juan Navarro
 */

#ifndef TIMER_H_
#define TIMER_H_

#include <stdint.h>

#define DEBUG_TIMER 0


// Initialize Timer/Counters.
// Start accurate software delays and Real Time Clock (RTC) timer.
void timer_init(void);

// Block during the specified time in milliseconds.
// Uses hardware timer to account for the time passed.
void timer_delay(uint16_t ms);

// Return the number of milliseconds since system startup.
uint32_t timer_systime(void);

void shortdelay(uint8_t cnt);

uint16_t timer_mscountdown_1;

#endif // TIMER_H_
