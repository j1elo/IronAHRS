/*
 * magnetometer_hmc5883l.c
 *
 * Created: 29/06/2012 16:56:05
 *  Author: Juan
 */

#include "sensor.h"
#include "magnetometer_hmc5883l.h"
#include "board/avr_io.h"
#include "board/TWI_Master.h"
#include <stdio.h>


#define ADDR_I2C 0x1E // I2C Address for 7-bit operations.
#define DEVICE_ID1 0x48 // Device's unique identifier.
#define DEVICE_ID2 0x34 // Device's unique identifier.
#define DEVICE_ID3 0x33 // Device's unique identifier.

// Register Addresses.
#define REG_CONFA  0 // Configuration Register A.
#define REG_CONFB  1 // Configuration Register B.
#define REG_MODE   2 // Mode Register.
#define REG_DATAXH 3 // X-Axis Data MSB.
#define REG_DATAXL 4 // X-Axis Data LSB.
#define REG_DATAZH 5 // Z-Axis Data MSB.
#define REG_DATAZL 6 // Z-Axis Data LSB.
#define REG_DATAYH 7 // Y-Axis Data MSB.
#define REG_DATAYL 8 // Y-Axis Data LSB.
#define REG_IDA   10 // Identification Register A.
#define REG_IDB   11 // Identification Register B.
#define REG_IDC   12 // Identification Register C.


// Read from the specified register.
// Reads 'nbytes' consecutive bytes and stores them in 'data'.
// Return TWI_NO_ERROR          if everything was ok.
//        TWI_TIMEOUT           if there was a timeout while waiting.
//        TWI_TRANSACTION_ERROR if there was a problem i.e. there was not device on the bus with that address.
uint8_t mag_read(uint8_t reg_address, uint8_t* data, uint8_t nbytes);

// Write 1 byte value into the specified register.
uint8_t mag_write(uint8_t reg_address, uint8_t data);


bool mag_check(void)
{
	// Switch to Continuous-Measurement mode.
	if (TWI_NO_ERROR != mag_write(REG_MODE, 0x00)) {
		// First write always fails, just check TWI status code.
		if (TWI_MTX_ADR_NACK == TWI_Get_State_Info()) {
			printf("SENSOR ERROR: mag_check(): wrong status\r\n");
			return false;
		}
	}

	uint8_t buffer[3] = {0,0,0};
	if (TWI_NO_ERROR != mag_read(REG_IDA, buffer, 3)) {
		printf("SENSOR ERROR: mag_check(): mag_read()\r\n");
		return false;
	}
#if DEBUG_SENSOR
	printf("SENSOR: mag_check(): 0x%X,0x%X,0x%X (should be 0x%X,0x%X,0x%X)\r\n",
		buffer[0], buffer[1], buffer[2],
		DEVICE_ID1, DEVICE_ID2, DEVICE_ID3);
#endif
	return (DEVICE_ID1 == buffer[0]) &&
		(DEVICE_ID2 == buffer[1]) &&
		(DEVICE_ID3 == buffer[2]);
}

void mag_init(mag_range_t range, mag_rate_t rate)
{
	// Some registers need a specific initialization:
	// REG_MODE bit 7 must be cleared -> done in mag_check();
	// REG_CONFA bit 7 must be cleared -> done in mag_set_rate().
	mag_set_range(range);
	mag_set_rate(rate);
}

void mag_set_range(mag_range_t range)
{
	switch (range) {
	case MAG_RANGE_08:
		mag_write(REG_CONFB, 0x00);
		break;
	case MAG_RANGE_13:
		mag_write(REG_CONFB, 0x20);
		break;
	case MAG_RANGE_19:
		mag_write(REG_CONFB, 0x40);
		break;
	case MAG_RANGE_25:
		mag_write(REG_CONFB, 0x60);
		break;
	case MAG_RANGE_40:
		mag_write(REG_CONFB, 0x80);
		break;
	case MAG_RANGE_47:
		mag_write(REG_CONFB, 0xA0);
		break;
	case MAG_RANGE_56:
		mag_write(REG_CONFB, 0xC0);
		break;
	case MAG_RANGE_81:
		mag_write(REG_CONFB, 0xE0);
		break;
	}
}

void mag_set_rate(mag_rate_t rate)
{
	switch (rate) {
	case MAG_RATE_3:
		mag_write(REG_CONFA, 0x04);
		break;
	case MAG_RATE_15:
		mag_write(REG_CONFA, 0x10);
		break;
	case MAG_RATE_30:
		mag_write(REG_CONFA, 0x14);
		break;
	case MAG_RATE_75:
		mag_write(REG_CONFA, 0x18);
		break;
	}
}

void mag_read_axis(int16_t axis[3])
{
	uint8_t buffer[6] = {0,0,0,0,0,0};

	if (TWI_NO_ERROR != mag_read(REG_DATAXH, buffer, 6)) {
		printf("SENSOR ERROR: mag_read_axis(): mag_read()\r\n");
		return;
	}

	// The HMC5883L gives the MSB first, then the LSB. Also, Y and Z are switched.
	axis[0] = (buffer[0] << 8) + buffer[1]; // X axis.
	axis[1] = (buffer[4] << 8) + buffer[5]; // Y axis.
	axis[2] = (buffer[2] << 8) + buffer[3]; // Z axis.

#if DEBUG_SENSOR
	printf("SENSOR: mag_read_axis(): X=%d, Y=%d, Z=%d\r\n", axis[0], axis[1], axis[2]);
#endif
}

uint8_t mag_read(uint8_t reg_address, uint8_t* data, uint8_t nbytes)
{
	uint8_t rc;

	// Set device's address pointer to the specified register address.
	rc = TWI_Start_Write(ADDR_I2C, &reg_address, 1);
	if (TWI_NO_ERROR != rc) {
		printf("SENSOR ERROR: mag_read(): TWI_Start_Write() rc=%u\r\n", rc);
		return rc;
	}

	// Now read the 'nbytes' consecutive registers and store its values.
	rc = TWI_Start_Read(ADDR_I2C, nbytes);
	if (TWI_NO_ERROR != rc) {
		printf("SENSOR ERROR: mag_read(): TWI_Start_Read() rc=%u\r\n", rc);
		return rc;
	}
	rc = TWI_Get_Data_From_Transceiver(data, nbytes);
	if (TWI_NO_ERROR != rc) {
		printf("SENSOR ERROR: mag_read(): TWI_Get_Data_From_Transceiver() rc=%u\r\n", rc);
		return rc;
	}

	return TWI_NO_ERROR;
}

uint8_t mag_write(uint8_t reg_address, uint8_t data)
{
	uint8_t rc;
	uint8_t buffer[2];

	buffer[0] = reg_address;
	buffer[1] = data;
	rc = TWI_Start_Write(ADDR_I2C, buffer, 2);
	if (TWI_NO_ERROR != rc) {
		//printf("SENSOR ERROR: mag_write(): TWI_Start_Write() rc=%u\r\n", rc);
		return rc;
	}

	return TWI_NO_ERROR;
}
