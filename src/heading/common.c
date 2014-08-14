/*
 * common.c
 *
 * Created: 12/09/2012 17:02:07
 * Author: Juan Navarro
 */

#include "common.h"
#include <stdint.h>
#include <math.h>

// Accelerometer calibration values.
#define ACC_BIAS_X ((ACC_MIN_X + ACC_MAX_X) / 2.0)
#define ACC_BIAS_Y ((ACC_MIN_Y + ACC_MAX_Y) / 2.0)
#define ACC_BIAS_Z ((ACC_MIN_Z + ACC_MAX_Z) / 2.0)
#define ACC_SCALE_X (ACC_SENSITIVITY / (ACC_MAX_X - ACC_BIAS_X))
#define ACC_SCALE_Y (ACC_SENSITIVITY / (ACC_MAX_Y - ACC_BIAS_Y))
#define ACC_SCALE_Z (ACC_SENSITIVITY / (ACC_MAX_Z - ACC_BIAS_Z))

// Magnetometer calibration values.
#define MAG_BIAS_X ((MAG_MIN_X + MAG_MAX_X) / 2.0)
#define MAG_BIAS_Y ((MAG_MIN_Y + MAG_MAX_Y) / 2.0)
#define MAG_BIAS_Z ((MAG_MIN_Z + MAG_MAX_Z) / 2.0)
#define MAG_SCALE_X (MAG_SENSITIVITY / (MAG_MAX_X - MAG_BIAS_X))
#define MAG_SCALE_Y (MAG_SENSITIVITY / (MAG_MAX_Y - MAG_BIAS_Y))
#define MAG_SCALE_Z (MAG_SENSITIVITY / (MAG_MAX_Z - MAG_BIAS_Z))

#if MAG_EXTENDED_CALIBRATION
// Extended magnetometer calibration: compensates hard & soft iron errors.
//const double magn_ellipsoid_center[3] = {0, 0, 0};
//const double magn_ellipsoid_transform[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
double mag_ellipsoid_center[3] = {-114.147, -89.2064, 8.35645};
double mag_ellipsoid_transform[3][3] = {{0.892748, 0.00218680, 0.00723185}, {0.00218680, 0.924024, -0.0173300}, {0.00723185, -0.0173300, 0.995624}};
#endif

// Pre-computed values used in deg_to_rad() and rad_to_deg().
#define CONST_DEG2RAD (M_PI / 180.0)
#define CONST_RAD2DEG (180.0 / M_PI)


double deg_to_rad(double degrees)
{
	// TODO check that CONST_DEG2RAD is actually precomputed at compilation time.
	return degrees * CONST_DEG2RAD;
}

double rad_to_deg(double radians)
{
	// TODO check that CONST_RAD2DEG is actually precomputed at compilation time.
	return radians * CONST_RAD2DEG;
}

// Multiply 3x3 matrix with 3x1 vector: out = a * b.
// No in-place: 'out' must be different from 'b'.
static void matrix_vector_multiply(double out[3], double a[3][3], double b[3])
{
	for (uint8_t row = 0; row < 3; row++) {
		out[row] = a[row][0] * b[0] + a[row][1] * b[1] + a[row][2] * b[2];
	}
}

void sensor_unbias_scale(double out_AccGyroMag_calib[3][3], /*const*/ double in_AccGyroMag_raw[3][3])
{
	// Compensate accelerometer bias and apply scaling.
	out_AccGyroMag_calib[0][0] = (in_AccGyroMag_raw[0][0] - ACC_BIAS_X) * ACC_SCALE_X;
	out_AccGyroMag_calib[0][1] = (in_AccGyroMag_raw[0][1] - ACC_BIAS_Y) * ACC_SCALE_Y;
	out_AccGyroMag_calib[0][2] = (in_AccGyroMag_raw[0][2] - ACC_BIAS_Z) * ACC_SCALE_Z;

#if 1
	// DEV: this includes unbias and scaling.
	// Compensate gyroscope bias and convert to radians.
	out_AccGyroMag_calib[1][0] = deg_to_rad((in_AccGyroMag_raw[1][0] - GYRO_AVG_BIAS_X) * GYRO_GAIN);
	out_AccGyroMag_calib[1][1] = deg_to_rad((in_AccGyroMag_raw[1][1] - GYRO_AVG_BIAS_Y) * GYRO_GAIN);
	out_AccGyroMag_calib[1][2] = deg_to_rad((in_AccGyroMag_raw[1][2] - GYRO_AVG_BIAS_Z) * GYRO_GAIN);
#else
	// DEV: this is the original Razor way. It scales later in razor.c::Matrix_update().
	// Original Razor gyroscope bias compensation. Doesn't scale and convert to radians.
	out_AccGyroMag_calib[1][0] = in_AccGyroMag_raw[1][0] - GYRO_AVG_BIAS_X;
	out_AccGyroMag_calib[1][1] = in_AccGyroMag_raw[1][1] - GYRO_AVG_BIAS_Y;
	out_AccGyroMag_calib[1][2] = in_AccGyroMag_raw[1][2] - GYRO_AVG_BIAS_Z;
#endif

	// Compensate magnetometer bias and apply scaling.
#if MAG_EXTENDED_CALIBRATION
	//double mag[3] = {in_AccGyroMag_raw[6], in_AccGyroMag_raw[7], in_AccGyroMag_raw[8]};
	//
	//for (uint8_t i = 0; i < 3; i++) {
		//mag_tmp[i] = mag[i] - mag_ellipsoid_center[i];
	//}
	//
	double mag_tmp[3];
	mag_tmp[0] = in_AccGyroMag_raw[2][0] - mag_ellipsoid_center[0];
	mag_tmp[1] = in_AccGyroMag_raw[2][1] - mag_ellipsoid_center[1];
	mag_tmp[2] = in_AccGyroMag_raw[2][2] - mag_ellipsoid_center[2];
	matrix_vector_multiply(out_AccGyroMag_calib[2], mag_ellipsoid_transform, mag_tmp);
#else
	out_AccGyroMag_calib[2][0] = (in_AccGyroMag_raw[2][0] - MAG_BIAS_X) * MAG_SCALE_X;
	out_AccGyroMag_calib[2][1] = (in_AccGyroMag_raw[2][1] - MAG_BIAS_Y) * MAG_SCALE_Y;
	out_AccGyroMag_calib[2][2] = (in_AccGyroMag_raw[2][2] - MAG_BIAS_Z) * MAG_SCALE_Z;
#endif
}
