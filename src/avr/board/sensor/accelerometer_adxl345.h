/*
 * accelerometer_adxl345.h
 *
 * Created: 29/06/2012 14:13:57
 * Author: Juan Navarro
 */

#ifndef ACCELEROMETER_ADXL345_H_
#define ACCELEROMETER_ADXL345_H_

#include <stdint.h>
#include <stdbool.h>


typedef enum {
	ACC_RANGE_2, // ± 2 g range.
	ACC_RANGE_4, // ± 4 g range.
	ACC_RANGE_8, // ± 8 g range.
	ACC_RANGE_16 // ± 16 g range.
} acc_range_t;

// Accelerometer Output Data Rate (ODR) and bandwidth.
typedef enum {
	ACC_RATE_50,  //  50 Hz rate, 25 Hz bandwidth.
	ACC_RATE_100, // 100 Hz rate, 50 Hz bandwidth.
	ACC_RATE_200, // 200 Hz rate, 100 Hz bandwidth.
	ACC_RATE_400, // 400 Hz rate, 200 Hz bandwidth.
	ACC_RATE_800  // 800 Hz rate, 400 Hz bandwidth.
} acc_rate_t;

//typedef enum {
	//ACC_AXIS_X,
	//ACC_AXIS_Y,
	//ACC_AXIS_Z
//} acc_axis_t;

// Check the presence of the ADXL345 chip.
bool acc_check(void);

// Configure and start sensor measurement.
void acc_init(acc_range_t range, acc_rate_t rate);

// Configure the accelerometer measurement range.
// Always sets Full Resolution mode: output resolution is 13 bits,
// maintaining a 4 mg/LSB scale factor.
// Always sets right-justified mode with sign extension.
void acc_set_range(acc_range_t range);

// Configure the accelerometer Output Data Rate (ODR) and bandwidth.
void acc_set_rate(acc_rate_t rate);

// Read acceleration values on the specified axis.
void acc_read_axis(int16_t axis[3]);

#endif // ACCELEROMETER_ADXL345_H_
