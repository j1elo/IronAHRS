/*
 * timer.c
 *
 * Created: 29/06/2012 17:43:14
 * Author: Juan Navarro
 */

#include "timer.h"
#include "timer_delay_x.h"
#include "avr_io.h"
#include <avr/interrupt.h>
#if DEBUG_TIMER
#include <stdio.h>
#endif

# define nop() __asm__ __volatile__ ("nop" ::)

// Pre-computed values used in timer_systime().
enum {
	MS_SECOND = 1000UL,
	MS_MINUTE = 1000UL * 60UL,
	MS_HOUR   = 1000UL * 60UL * 60UL,
	MS_DAY    = 1000UL * 60UL * 60UL * 24UL
};


// Start 8-bit Timer/Counter0.
// This timer is driven by an external 11.0592 MHz clock, connected to pins XTAL1/2.
// Provides accurate software delays in milliseconds.
static void init_timer0(void);

// Start 8-bit Timer/Counter2.
// This timer is driven by an external 32.768 KHz clock, connected to pins PORTC6/7 (pins TOSC1/2).
// Provides Real Time Clock (RTC) with low power consumption.
static void init_timer2(void);

// Function: Timer/Counter0 Compare Match A - interrupt handler.
// Timer/Counter0 interrupts every 1 msec.
// Decrement countdown timers - other routines can control timing to
// within 1 msec by monitoring these software timers.
// ISR(TIMER0_COMPA_vect);

// Function: Timer/Counter2 Compare Match A - interrupt handler.
// Real Time Clock interrupts every 4msec.
// Decrement countdown timers.
// ISR(TIMER2_COMPA_vect);

uint16_t delay_mscountdown;
uint8_t rtc_clockcnt, second, minute, hour, day, month;
uint16_t year;


void timer_init(void)
{
	init_timer0();
	init_timer2();
}

void timer_delay(uint16_t ms)
{
	delay_mscountdown = ms;

	// Wait for the specified time.
	// Use "volatile" to avoid removal by optimization.
	while (*(volatile uint16_t *)&delay_mscountdown > 0); // BLOCK here.
}

uint32_t timer_systime(void)
{
	// Be VERY CAREFUL with integer overflows here.

	// Milliseconds = rtc_clockcnt * (Timer/Counter2 time measurement).

#if DEBUG_TIMER
	uint32_t ret = (((uint32_t)rtc_clockcnt) << 2)  // Value * 4 ms.
		+ ((uint32_t)second) * MS_SECOND
		+ ((uint32_t)minute) * MS_MINUTE;
		//+ ((uint32_t)hour) * MS_HOUR
		//+ ((uint32_t)day) * MS_DAY;
	printf("TIMER: timer_systime(): %lu ms\r\n", ret);
	return ret;
#else
	return (((uint32_t)rtc_clockcnt) << 2)  // Value * 4 ms.
		+ ((uint32_t)second) * MS_SECOND
		+ ((uint32_t)minute) * MS_MINUTE;
		//+ ((uint32_t)hour) * MS_HOUR
		//+ ((uint32_t)day) * MS_DAY;
#endif
}

void shortdelay(uint8_t cnt)
{
	// TODO: check SPI implementation as this is probably not really needed here.

	//for (uint8_t i = 0; i < cnt; i++) {
		//nop();
	//}

	_delay_ns(200);
	//_delay_us(500);
}

//void shortdelay(uint8_t cnt)
//{
	//volatile unsigned char inbyte;
	//int i;
	//
	//for (i = 0; i < cnt; i++)  {
		//inbyte = 0x55;
	//}
//}

static void init_timer0(void)
{
	delay_mscountdown = 0;
	timer_mscountdown_1 = 0;

	// Choosing prescaler value:
	// Objective: measurement of 1 ms: T1 = 0.001 s.
	// Resolution: 8 bits: R1 = 256 ticks.
	// Frequency: F = 11059200 Hz (ticks/s).
	// Scaling: K1 = F*T1/R1 = 43.2.
	// Best fit prescaler: K2 = 64.
	// Equivalent resolution: R2 = F*T1/K2 = 172.8 = 173.
	// Actual measurement: T2 = R2*K2/F = 1.0012 ms.

	// Set initial value.
	TCNT0 = 0;

	// Compare Output Mode:
	// Normal port operation, pins OC0A/B disconnected.
	BIT_CLEAR(TCCR0A, COM0A1);
	BIT_CLEAR(TCCR0A, COM0A0);
	BIT_CLEAR(TCCR0A, COM0B1);
	BIT_CLEAR(TCCR0A, COM0B0);

	// Waveform Generation Mode:
	// Clear Timer on Compare Match (CTC), non-PWM Mode.
	BIT_CLEAR(TCCR0B, WGM02);
	BIT_SET(TCCR0A, WGM01);
	BIT_CLEAR(TCCR0A, WGM00);
	OCR0A = 173;  // Timer resolution.

	// Clock Select:
	// System clock frequency (clkIO) / 64.
	// This command enables the timer.
	BIT_CLEAR(TCCR0B, CS02);
	BIT_SET(TCCR0B, CS01);
	BIT_SET(TCCR0B, CS00);

	// Timer/Counter0 Output Compare Match A Interrupt Enable.
	BIT_SET(TIMSK0, OCIE0A);

	/*
	// Set mode 2 - reset counter on compare match, clk/64
	TCCR0A = (1<<WGM01);
	TCCR0B = (1<<WGM01)|(1<<CS01)|(1<<CS00);

	// Reset counter after 1 millisecond (11.0592 MHz clock).
	// (11059200 Hz * 1 ms) / (64 * 1000 ms) = 172.8 Hz.
	OCR0A = 173;

	// Enable compare match interrupt vector
	TIMSK0 |= (1<<OCIE0A);
	*/
}

static void init_timer2(void)
{
	rtc_clockcnt = 0;
	second = minute = hour = day = month = year = 0;

	// Choosing prescaler value:
	// Objective: measurement of 4 ms: T1 = 0.004 s.
	// Resolution: 8 bits: R1 = 256 ticks.
	// Frequency: F = 32768 Hz (ticks/s).
	// Scaling: K1 = F*T1/R1 = 0.512.
	// Best fit prescaler: K2 = 1.
	// Equivalent resolution: R2 = F*T1/K2 = 131.072 = 131.
	// Actual measurement: T2 = R2*K2/F = 3.9978 ms.

	// A 32.768 kHz crystal has a stabilization time
	// up to one second after Power-up.
	_delay_s(1);

	// Warning: when switching between asynchronous and synchronous clocking,
	// the contents of TCNT2, OCR2A, OCR2B, TCCR2A and TCCR2B might be corrupted.
	// Safe procedure for switching clock source:
	// 1. Disable the Timer/Counter2 interrupts by clearing OCIE2x and TOIE2.
	// 2. Select clock source by setting AS2 as appropriate.
	// 3. Write new values to TCNT2, OCR2x, and TCCR2x.
	// 4. To switch to asynchronous operation: Wait for TCN2UB, OCR2xUB, and TCR2xUB.
	// 5. Clear the Timer/Counter2 Interrupt Flags.
	// 6. Enable interrupts, if needed.

	// 1. Disable interrupts.
	BIT_CLEAR(TIMSK2, OCIE2B);
	BIT_CLEAR(TIMSK2, OCIE2A);
	BIT_CLEAR(TIMSK2, TOIE2);

	// 2. Asynchronous Timer/Counter2:
	// Enable external crystal oscillator connected to the TOSC1 pin.
	BIT_SET(ASSR, AS2);

	// 3. Set initial values.
	TCNT2 = 0;

	// Compare Output Mode:
	// Normal port operation, pins OC2A/B disconnected.
	BIT_CLEAR(TCCR2A, COM2A1);
	BIT_CLEAR(TCCR2A, COM2A0);
	BIT_CLEAR(TCCR2A, COM2B1);
	BIT_CLEAR(TCCR2A, COM2B0);

	// Waveform Generation Mode:
	// Clear Timer on Compare Match (CTC), non-PWM Mode.
	BIT_CLEAR(TCCR2B, WGM22);
	BIT_SET(TCCR2A, WGM21);
	BIT_CLEAR(TCCR2A, WGM20);
	OCR2A = 131;  // Timer resolution.

	// Clock Select:
	// External clock frequency (clkT2S) / 1.
	// This command enables the timer.
	BIT_CLEAR(TCCR2B, CS22);
	BIT_CLEAR(TCCR2B, CS21);
	BIT_SET(TCCR2B, CS20);

	// 4. Wait for busy registers.
	while (ASSR & (BIT(TCN2UB) | BIT(OCR2AUB) | BIT(OCR2BUB) | BIT(TCR2AUB) | BIT(TCR2BUB)));

	// 5. Timer/Counter2 Output Compare Match A Interrupt Enable.
	BIT_SET(TIMSK2, OCIE2A);

	/*
	TCNT2 = 0;
	ASSR = (1<<AS2); // Select external real time clock oscillator

	// Reset counter after 4 milliseconds (32.768 KHz clock).
	// (32756 Hz * 4 ms) / (1000 ms) = 131.024 Hz.
	OCR2A = 131;

	TCCR2A = (1<<WGM21); // Reload counter on match interrupt - pre-scalar set to 0
	TCCR2B = (1<<CS20);
	TIMSK2 |= (1<<OCIE2A); // setup interrupts for real time clock
	*/
}

ISR(TIMER0_COMPA_vect)
{
	if (delay_mscountdown > 0) --delay_mscountdown;
	if (timer_mscountdown_1 > 0) --timer_mscountdown_1;
}

ISR(TIMER2_COMPA_vect)
{
	if (++rtc_clockcnt >= 250) {
		rtc_clockcnt = 0;
		if (++second == 60) {
			second = 0;
			if (++minute == 60) {
				minute = 0;
				if (++hour == 24) {
					hour = 0;
					++day;
					if (day == 32) {
						day = 1;
						++month;
					}
					else if (day == 31) {
						if ((month == 4) || (month == 6) || (month == 9) || (month == 11)) {
							day = 1;
							++month;
						}
					}
					else if (day == 30) {
						if (month == 2) {
							day = 1;
							++month;
						}
					}
					else if (day == 29) {
						// Check for leap year.
						// TODO move to function and write tests.
						if ((month == 2) && (((year % 100) ? (year % 4) : (year % 400)))) {
							day = 1;
							++month;
						}
					}
					if (month == 13) {
						month = 1;
						++year;
					}
				}
			}
		}
	}
}
