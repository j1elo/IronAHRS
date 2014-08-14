/*
 * magnetometer_hmc5883l.h
 *
 * Created: 29/06/2012 16:56:14
 * Author: Juan Navarro
 */

#ifndef MAGNETOMETER_HMC5883L_H_
#define MAGNETOMETER_HMC5883L_H_

#include <stdint.h>
#include <stdbool.h>


// Magnetometer sensor range: [-2048, 2047].
typedef enum {
	MAG_RANGE_08, // ± 0.88 Gauss range; 1370 LSB/Gauss.
	MAG_RANGE_13, // ± 1.3 Gauss range; 1090 LSB/Gauss. DEFAULT.
	MAG_RANGE_19, // ± 1.9 Gauss range; 820 LSB/Gauss.
	MAG_RANGE_25, // ± 2.5 Gauss range; 660 LSB/Gauss.
	MAG_RANGE_40, // ± 4.0 Gauss range; 440 LSB/Gauss.
	MAG_RANGE_47, // ± 4.7 Gauss range; 390 LSB/Gauss.
	MAG_RANGE_56, // ± 5.6 Gauss range; 330 LSB/Gauss.
	MAG_RANGE_81  // ± 8.1 Gauss range; 230 LSB/Gauss.
} mag_range_t;

// Magnetometer Output Data Rate (ODR).
typedef enum {
	MAG_RATE_3,  //  3 Hz rate.
	MAG_RATE_15, // 15 Hz rate. DEFAULT.
	MAG_RATE_30, // 30 Hz rate.
	MAG_RATE_75  // 75 Hz rate.
} mag_rate_t;


// Check the presence of the HMC5883L chip.
bool mag_check(void);

// Configure and start sensor measurement.
void mag_init(mag_range_t range, mag_rate_t rate);

// Configure the magnetometer measurement range.
void mag_set_range(mag_range_t range);

// Configure the magnetometer Output Data Rate (ODR) and bandwidth.
void mag_set_rate(mag_rate_t rate);

// Read magnetometer values on the specified axis.
void mag_read_axis(int16_t axis[3]);

#endif // MAGNETOMETER_HMC5883L_H_
