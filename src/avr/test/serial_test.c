/*
 * serial_test.c
 *
 * Created: 18/09/2012 1:16:40
 * Author: Juan Navarro
 */

#include "tests.h"
#include "board/serial.h"
#include <avr/interrupt.h>
#include <stdint.h>
#include <stdlib.h>
#include <assert.h>


void serial_test(void)
{
	serial_init(BAUDRATE115200);
	sei(); // Set Global Interrupt Enable.

	assert(0 == serial_available());

	uint8_t buffer[8] = {'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H'};
	assert(1 == serial_write(buffer, 1));

	assert(0 == serial_available());
	assert(0 == serial_available());
	assert(0 == serial_available());
	assert(0 == serial_available());
	assert(0 == serial_available());
}
