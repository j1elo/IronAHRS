/*
 * accelerometer_adxl345.c
 *
 * Created: 29/06/2012 14:13:49
 *  Author: Juan
 */

#include "sensor.h"
#include "accelerometer_adxl345.h"
#include "board/avr_io.h"
#include "board/IMU6410_SPI.h"
#include "board/timer.h"
#include <stdio.h>


#define DEVICE_ID 0xE5 // Device's unique identifier.

// Register Addresses.
// Could be OR-ed with one or more Register Address Modifiers.
#define REG_DEVID       0x00 // Device ID.
#define REG_BW_RATE     0x2C // Data rate and power mode control.
#define REG_POWER_CTL   0x2D // Power-saving features control.
#define REG_INT_ENABLE  0x2E // Interrupt enable control.
#define REG_DATA_FORMAT 0x31 // Data format control.
#define REG_DATAXL      0x32 // X-Axis Data LSB.
#define REG_DATAXH      0x33 // X-Axis Data MSB.
#define REG_DATAYL      0x34 // Y-Axis Data LSB.
#define REG_DATAYH      0x35 // Y-Axis Data MSB.
#define REG_DATAZL      0x36 // Z-Axis Data LSB.
#define REG_DATAZH      0x37 // Z-Axis Data MSB.

// Register Address Modifiers, for specific commands.
// The higher bits in an address specify whether the command is a Read
// instead of a Write, and if it is a single or multiple-bytes operation.
#define READ_ENABLE 0x80 // Enable Read command.
#define MB_ENABLE   0x40 // Enable MultiByte command.


// Set or clear Chip Select condition.
// "true": enables CS; "false" disables CS.
void acc_chip_select(bool select);

// Read from the specified register.
// Reads 'nbytes' consecutive bytes and stores them in 'data'.
void acc_read(uint8_t reg_address, uint8_t* data, uint8_t nbytes);

// Write 1 byte value into the specified register.
void acc_write(uint8_t reg_address, uint8_t data);


bool acc_check(void)
{
	uint8_t buffer = 0;
	acc_read(REG_DEVID, &buffer, 1);
#if DEBUG_SENSOR
	printf("SENSOR: acc_check(): 0x%X (should be 0x%X)\r\n", buffer, DEVICE_ID);
#endif
	return DEVICE_ID == buffer;
}

void acc_init(acc_range_t range, acc_rate_t rate)
{
	acc_set_range(range);
	acc_set_rate(rate);

	// Not sure why, code in IMU_Controller enables DATA_READY interrupt output.
	acc_write(REG_INT_ENABLE, 0x80);

	// Awake from standby and start sensor measurements.
	acc_write(REG_POWER_CTL, 0x08);
}

void acc_set_range(acc_range_t range)
{
	switch (range) {
	case ACC_RANGE_2:
		acc_write(REG_DATA_FORMAT, 0x08);
		break;
	case ACC_RANGE_4:
		acc_write(REG_DATA_FORMAT, 0x09);
		break;
	case ACC_RANGE_8:
		acc_write(REG_DATA_FORMAT, 0x0A);
		break;
	case ACC_RANGE_16:
		acc_write(REG_DATA_FORMAT, 0x0B);
		break;
	}
}

void acc_set_rate(acc_rate_t rate)
{
	switch (rate) {
	case ACC_RATE_50:
		acc_write(REG_BW_RATE, 0x09);
		break;
	case ACC_RATE_100:
		acc_write(REG_BW_RATE, 0x0A);
		break;
	case ACC_RATE_200:
		acc_write(REG_BW_RATE, 0x0B);
		break;
	case ACC_RATE_400:
		acc_write(REG_BW_RATE, 0x0C);
		break;
	case ACC_RATE_800:
		acc_write(REG_BW_RATE, 0x0D);
		break;
	}
}

void acc_read_axis(int16_t axis[3])
{
	uint8_t buffer[6] = {0,0,0,0,0,0};

	acc_read(REG_DATAXL, buffer, 6);

	// The ADXL345 gives the LSB first, then the MSB.
	axis[0] = (buffer[1] << 8) + buffer[0]; // X axis.
	axis[1] = (buffer[3] << 8) + buffer[2]; // Y axis.
	axis[2] = (buffer[5] << 8) + buffer[4]; // Z axis.

#if DEBUG_SENSOR
	printf("SENSOR: acc_read_axis(): X=%d, Y=%d, Z=%d\r\n", axis[0], axis[1], axis[2]);
#endif
}

void acc_chip_select(bool select)
{
	shortdelay(2);
	if (select)
		BIT_CLEAR(PORTB, PORTB1);
	else
		BIT_SET(PORTB, PORTB1);
	shortdelay(2);
}

void acc_read(uint8_t reg_address, uint8_t* data, uint8_t nbytes)
{
	reg_address |= READ_ENABLE;
	if (nbytes > 1) reg_address |= MB_ENABLE;

	acc_chip_select(true);
	SPI_MasterTransmit(reg_address);
	for (uint8_t i=0; i<nbytes; i++) {
		data[i] = SPI_MasterReceive();
	}
	acc_chip_select(false);
}

void acc_write(uint8_t reg_address, uint8_t data)
{
	acc_chip_select(true);
	SPI_MasterTransmit(reg_address);
	SPI_MasterTransmit(data);
	acc_chip_select(false);
}
