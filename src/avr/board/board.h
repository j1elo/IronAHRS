/*
 * board.h
 *
 * Created: 29/06/2012 17:13:12
 * Author: Juan Navarro
 */

#ifndef BOARD_H_
#define BOARD_H_

#include <stdbool.h>


// Initialize IMU6410 sensor board.
// All board systems get prepared for normal operation:
// - Serial port (USART).
// - I/O ports.
// - Internal I2C (aka. "TWI") and SPI buses.
// - Internal Timer/Counters.
void board_init(void);

// Enable or disable Red LED.
void board_red_led(bool enable);

// Enable or disable Green LED.
void board_green_led(bool enable);

// Run the software STK500v2 bootloader.
// The bootloader must be preloaded in the AVR flash.
// Once loaded, AVRdude application can be used for programming a new binary into the AVR flash memory.
void stk500v2_bootloader(void);

#endif // BOARD_H_
