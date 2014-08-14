/*
 * Sensors.c
 *
 * Created: 27/06/2012 18:21:40
 *  Author: Juan
 */

#include "sensor.h"
#include "board/TWI_Master.h"
#include "board/IMU6410_SPI.h"
//#include "board/timer.h"
#include <stdio.h>
#include <stdint.h>


void sensor_init(void)
{
	printf("\r\nCheck IMU6410 Sensor presence\r\n");

	TWI_Master_Initialize();

	Release_SPI();
	Init_SPI_Master();

	// Check for ADXL345.
	printf("Accelerometer ADXL345: ");
	if (acc_check()) {
		acc_init(ACC_RANGE_4, ACC_RATE_100);
		printf("FOUND\r\n");
	}
	else printf("NOT FOUND\r\n");

	// Check for L3G4200D.
	printf("Gyroscope L3G4200D: ");
	if (gyro_check()) {
		gyro_init(GYRO_RANGE_500, GYRO_RATE_100);
		printf("FOUND\r\n");
	}
	else printf("NOT FOUND\r\n");

	// Check for HMC5883L.
	printf("Magnetometer HMC5883L: ");
	if (mag_check()) {
		mag_init(MAG_RANGE_08, MAG_RATE_75);
		printf("FOUND\r\n");
	}
	else printf("NOT FOUND\r\n");
}
