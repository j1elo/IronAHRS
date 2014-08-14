/*
 * common.h
 *
 * Created: 12/09/2012 17:02:15
 * Author: Juan Navarro
 */

#ifndef COMMON_H_
#define COMMON_H_

#define AGM_DEFAULT_BIAS 1
#define MAG_DEFAULT_SCALING 1
#define MAG_EXTENDED_CALIBRATION 0

/* SENSOR CALIBRATION */
/* ****************** */
// How to calibrate? Read the tutorial at http://dev.qu.tu-berlin.de/projects/sf-razor-9dof-ahrs
// Put MIN/MAX and BIAS readings for your board here!

// "Acc X,Y,Z (min/max) = MIN_X/MAX_X  MIN_Y/MAX_Y  MIN_Z/MAX_Z".
// Default: -250.0 / +250.0
#if AGM_DEFAULT_BIAS
#define ACC_MIN_X -250.0
#define ACC_MAX_X  250.0
#define ACC_MIN_Y -250.0
#define ACC_MAX_Y  250.0
#define ACC_MIN_Z -250.0
#define ACC_MAX_Z  250.0
#else
#define ACC_MIN_X -261.0
#define ACC_MAX_X +266.0
#define ACC_MIN_Y -238.0
#define ACC_MAX_Y +284.0
#define ACC_MIN_Z -280.0
#define ACC_MAX_Z +246.0
#endif

// "Gyro x,y,z (current/average) = .../BIAS_X  .../BIAS_Y  .../BIAS_Z".
// Default: 0.0
#if AGM_DEFAULT_BIAS
#define GYRO_AVG_BIAS_X 0.0
#define GYRO_AVG_BIAS_Y 0.0
#define GYRO_AVG_BIAS_Z 0.0
#else
#define GYRO_AVG_BIAS_X +80.0
#define GYRO_AVG_BIAS_Y -30.0
#define GYRO_AVG_BIAS_Z -20.0
#endif

// "Mag X,Y,Z (min/max) = MIN_X/MAX_X  MIN_Y/MAX_Y  MIN_Z/MAX_Z".
// Default: -600.0 / +600.0
#if AGM_DEFAULT_BIAS
#define MAG_MIN_X -600.0
#define MAG_MAX_X  600.0
#define MAG_MIN_Y -600.0
#define MAG_MAX_Y  600.0
#define MAG_MIN_Z -600.0
#define MAG_MAX_Z  600.0
#else
#define MAG_MIN_X -750.0
#define MAG_MAX_X +522.0
#define MAG_MIN_Y -700.0
#define MAG_MAX_Y +530.0
#define MAG_MIN_Z -550.0
#define MAG_MAX_Z +580.0
#endif

// Current accelerometer config: full resolution. Sensitivity: 256 LSB/g.
// Current gyroscope config: 500 degrees per second. Gain: 0.0175 degrees/LSB.
// Current magnetometer config: 0.88 Gauss increments. Sensitivity: 1370 LSB/Gauss.
// Intensity of Earth's magnetic field at Madrid: around 0.445 Gauss.
// Source: http://en.wikipedia.org/wiki/Earth's_magnetic_field
// => Magnetometer increment for 0.445 Gauss: 609.65 LSB.
// TODO these are specific sensor parameter. Move to sensor implementation?
#define ACC_SENSITIVITY 256.0  // "1G reference" used for calibration and filters.
#define GYRO_GAIN 0.0175
#if MAG_DEFAULT_SCALING
	#define MAG_SENSITIVITY 100.0
#else
	#define MAG_SENSITIVITY 609.65
#endif


// Unit conversion.
double deg_to_rad(double degrees);
double rad_to_deg(double radians);

// Calibrate raw sensor readings.
void sensor_unbias_scale(double out_AccGyroMag_calib[3][3], /*const*/ double in_AccGyroMag_raw[3][3]);

#endif // COMMON_H_
