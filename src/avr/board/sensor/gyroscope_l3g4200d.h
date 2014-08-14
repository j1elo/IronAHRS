/*
 * gyroscope_l3g4200d.h
 *
 * Created: 29/06/2012 14:19:02
 * Author: Juan Navarro
 */

#ifndef GYROSCOPE_L3G4200D_H_
#define GYROSCOPE_L3G4200D_H_

#include <stdint.h>
#include <stdbool.h>


typedef enum {
	GYRO_RANGE_250, // ± 250 Degrees Per Second range.
	GYRO_RANGE_500, // ± 500 Degrees Per Second range.
	GYRO_RANGE_2000 // ± 2000 Degrees Per Second range.
} gyro_range_t;

// Gyroscope Output Data Rate (ODR).
typedef enum {
	GYRO_RATE_100, // 100 Hz rate.
	GYRO_RATE_200, // 200 Hz rate.
	GYRO_RATE_400, // 400 Hz rate.
	GYRO_RATE_800  // 800 Hz rate.
} gyro_rate_t;


// Check the presence of the L3G4200D chip.
bool gyro_check(void);

// Configure and start sensor measurement.
void gyro_init(gyro_range_t range, gyro_rate_t rate);

// Configure the gyroscope measurement range.
void gyro_set_range(gyro_range_t range);

// Configure the gyroscope Output Data Rate (ODR).
// Also awakes from standby and starts sensor measurements.
void gyro_set_rate(gyro_rate_t rate);

// Read rotation values on the specified axis.
void gyro_read_axis(int16_t axis[3]);

#endif // GYROSCOPE_L3G4200D_H_
