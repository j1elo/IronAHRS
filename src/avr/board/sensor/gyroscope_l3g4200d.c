/*
 * gyroscope_l3g4200d.c
 *
 * Created: 29/06/2012 14:18:49
 *  Author: Juan
 */

#include "sensor.h"
#include "gyroscope_l3g4200d.h"
#include "board/avr_io.h"
#include "board/IMU6410_SPI.h"
#include "board/timer.h"
#include <stdio.h>


#define DEVICE_ID 0xD3 // Device's unique identifier.

// Register Address.
// Could be OR-ed with one or more Register Address Modifiers.
#define REG_WHO_AM_I  0x0F // Device identification register.
#define REG_CTRL_REG1 0x20 // Device configuration register 1.
#define REG_CTRL_REG2 0x21 // Device configuration register 2.
#define REG_CTRL_REG4 0x23 // Device configuration register 4.
#define REG_CTRL_REG5 0x24 // Device configuration register 5.
#define REG_DATAXL    0x28 // X-Axis Data LSB.
#define REG_DATAXH    0x29 // X-Axis Data MSB.
#define REG_DATAYL    0x2A // Y-Axis Data LSB.
#define REG_DATAYH    0x2B // Y-Axis Data MSB.
#define REG_DATAZL    0x2C // Z-Axis Data LSB.
#define REG_DATAZH    0x2D // Z-Axis Data MSB.

// Register Address Modifiers, for specific commands.
// The higher bits in an address specify whether the command is a Read
// instead of a Write, and if it is a single or multiple-bytes operation.
#define READ_ENABLE 0x80 // Enable Read command.
#define MB_ENABLE   0x40 // Enable MultiByte command.


// Set or clear Chip Select condition.
// "select": "1" enables CS; "0" disables CS.
void gyro_chip_select(bool select);

// Read from the specified register.
// Reads 'nbytes' consecutive bytes and stores them in 'data'.
void gyro_read(uint8_t reg_address, uint8_t* data, uint8_t nbytes);

// Write 1 byte value into the specified register.
void gyro_write(uint8_t reg_address, uint8_t data);

// Configure High-pass & Low-pass filter Cut Off frequencies.
void gyro_set_filters();


bool gyro_check(void)
{
	uint8_t buffer = 0;
	gyro_read(REG_WHO_AM_I, &buffer, 1);
#if DEBUG_SENSOR
	printf("SENSOR: gyro_check(): 0x%X (should be 0x%X)\r\n", buffer, DEVICE_ID);
#endif
	return DEVICE_ID == buffer;
}

void gyro_init(gyro_range_t range, gyro_rate_t rate)
{
	gyro_set_range(range);
    shortdelay(2);
	gyro_set_rate(rate);
    shortdelay(2);
}

void gyro_set_range(gyro_range_t range)
{
	switch (range) {
	case GYRO_RANGE_250:
		gyro_write(REG_CTRL_REG4, 0x00);
		break;
	case GYRO_RANGE_500:
		gyro_write(REG_CTRL_REG4, 0x10);
		break;
	case GYRO_RANGE_2000:
		gyro_write(REG_CTRL_REG4, 0x20);
		break;
	}
}

void gyro_set_rate(gyro_rate_t rate)
{
	switch (rate) {
	case GYRO_RATE_100:
		gyro_write(REG_CTRL_REG1, 0x3F);
		break;
	case GYRO_RATE_200:
		gyro_write(REG_CTRL_REG1, 0x7F);
		break;
	case GYRO_RATE_400:
		gyro_write(REG_CTRL_REG1, 0xBF);
		break;
	case GYRO_RATE_800:
		gyro_write(REG_CTRL_REG1, 0xFF);
		break;
	}
}

void gyro_read_axis(int16_t axis[3])
{
	uint8_t buffer[6] = {0,0,0,0,0,0};

	gyro_read(REG_DATAXL, buffer, 6);

	// The L3G4200D gives the LSB first, then the MSB.
	axis[0] = (buffer[1] << 8) + buffer[0]; // X axis.
	axis[1] = (buffer[3] << 8) + buffer[2]; // Y axis.
	axis[2] = (buffer[5] << 8) + buffer[4]; // Z axis.

#if DEBUG_SENSOR
	printf("SENSOR: gyro_read_axis(): X=%d, Y=%d, Z=%d\r\n", axis[0], axis[1], axis[2]);
#endif
}

void gyro_chip_select(bool select)
{
	shortdelay(2);
	if (select)
		BIT_CLEAR(PORTB, PORTB2);
	else
		BIT_SET(PORTB, PORTB2);
	shortdelay(2);
}

void gyro_read(uint8_t reg_address, uint8_t* data, uint8_t nbytes)
{
	reg_address |= READ_ENABLE;
	if (nbytes > 1) reg_address |= MB_ENABLE;

	gyro_chip_select(true);
	SPI_MasterTransmit(reg_address);
	for (uint8_t i=0; i<nbytes; i++) {
		data[i] = SPI_MasterReceive();
	}
	gyro_chip_select(false);
}

void gyro_write(uint8_t reg_address, uint8_t data)
{
	gyro_chip_select(true);
	SPI_MasterTransmit(reg_address);
	SPI_MasterTransmit(data);
	gyro_chip_select(false);
}

void gyro_set_filters()
{
    gyro_write(REG_CTRL_REG2, 0x29);  // High-pass filter setup.
    shortdelay(2);
    gyro_write(REG_CTRL_REG5, 0x13);  // High-pass filter enable.
    shortdelay(2);
}