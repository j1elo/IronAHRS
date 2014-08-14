/*
 * Serial.h
 *
 * Created: 27/06/2012 18:34:15
 * Author: Juan Navarro
 */

#ifndef SERIAL_H_
#define SERIAL_H_

#define DEBUG_SERIAL 0
#define DEBUG_SERIAL_WRITE 0

#include <stdint.h>

//  Baud rates for 11.0592 MHz
//TODO change using this: http://www.nongnu.org/avr-libc/user-manual/group__util__setbaud.html
#define  BAUDRATE9600    71
#define  BAUDRATE19200	 35
#define  BAUDRATE38400   17
#define  BAUDRATE57600   11
#define  BAUDRATE115200   5
#define  BAUDRATE230400   2


void serial_init(uint8_t baud);

// Check if there is any incoming serial data available.
// Return the number of bytes available for reading from the serial port.
uint16_t serial_available(void);

// Read incoming serial data.
// Block until data is available and return the first byte.
uint8_t serial_read(void);

// Write data to the serial port.
// Return the number of bytes written.
uint16_t serial_write(const uint8_t* data, uint16_t size);

#endif // SERIAL_H_
