/*
 * board.c
 *
 * Created: 29/06/2012 17:13:19
 *  Author: Juan
 */

#include "board.h"
#include "serial.h"
#include "eeprom.h"
#include "timer.h"
#include "sensor/sensor.h"
#include "avr_io.h"
#include <avr/interrupt.h>
#include <stdio.h>


// Function: Catch-all interrupt handler.
// Called if an unexpected interrupt occurs.
// ISR(BADISR_vect);

void board_init(void)
{
	timer_init();  // Init AVR Timer/Counters for software RTC and delays.
	serial_init(BAUDRATE115200);  // Init AVR USART for serial communications.

	// Unload unique byte so bootloader doesn't keep running after being started in next power up.
	if (eeprom_read(1) == 0x55) {
		eeprom_write(1, 0x00);
	}

	// Setup Port A
	//  Setup the Analog to Digital converter
	DDRA = 0x00;  // All inputs
	// TODO - convert to background interrupt process
	// TODO - setup Analog

	// Setup Port B
	//   PB0 - NC
	//   PB1 - ADXL345 CS
	//   PB2 - L3G4200 CS
	//   PB3 - Chip Select for uSD on IMU6410
	//   PB4 - SS - Chip select for uSD on IMU6420
	DDRB = 0x1E;
	PORTB = 0x1E;

	// Setup Port C
	//    PC0 is SCL - must be output
	//    PC1 is SDA
	//    PC2 XCLR BMP085
	//    PC3,4,5 Digital IO
	DDRC = 0x05;
	PORTC = 0x05;

	// Setup Port D
	//   IMU6410 Pin assignments:
	//     PD2 - Mag DRDY
	//     PD6 - Gyro INT2
	//     PD7 - Accel INT2
	//     PD5 - Red LED
	//     PD4 - Green LED
    DDRD = 0x30;

	// Turn LEDs off.
	board_red_led(false);
	board_green_led(false);

	sei(); // Set Global Interrupt Enable.

	timer_delay(50); // Give sensors enough time to start (not needed cos timer_init() takes 1 second).
	sensor_init();
	timer_delay(50); // Give sensors enough time to collect data.
}

void board_red_led(bool enable)
{
	if (enable)
		BIT_CLEAR(PORTD, PIND5); // Low output to turn LED on.
	else
		BIT_SET(PORTD, PIND5); // High output to turn LED off.
}

void board_green_led(bool enable)
{
	if (enable)
		BIT_CLEAR(PORTD, PIND4); // Low output to turn LED on.
	else
		BIT_SET(PORTD, PIND4); // High output to turn LED off.
}

void stk500v2_bootloader(void)
{
	printf("Launching STK500v2 bootloader...\r\n");

	board_green_led(false);

	// Load unique byte so bootloader keeps running after being started.
	eeprom_write(1, 0x55); // Load unique byte so hold bootloader.

	//  Wait for completion of write
	timer_delay(30);

	// Jump to first address of bootloader code.
	asm("jmp 0x1f000");
}

ISR(BADISR_vect)
{
	printf("ERROR: unexpected interrupt, madafaka!!\r\n");
}
