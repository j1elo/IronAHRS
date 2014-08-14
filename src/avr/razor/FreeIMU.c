#include "razor.h"
#include "FreeIMU.h"
#include "MadgwickAHRS.h"
#include <math.h>

#define twoKpDef  (2.0 * 0.5) // 2 * proportional gain
#define twoKiDef  (2.0 * 0.1) // 2 * integral gain

// quaternion of sensor frame relative to auxiliary frame
static double q0 = 1.0, q1 = 0.0, q2 = 0.0, q3 = 0.0;

double exInt = 0.0;
double eyInt = 0.0;
double ezInt = 0.0;
volatile double twoKp = twoKpDef;
volatile double twoKi = twoKiDef;
volatile double integralFBx = 0.0,  integralFBy = 0.0, integralFBz = 0.0;

double iq0, iq1, iq2, iq3;
double freeIMUsampleFreq; // half the sample period expressed in seconds

void FreeIMUAHRSupdate(double gx, double gy, double gz, double ax, double ay, double az, double mx, double my, double mz)
{
	double recipNorm;
	double q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
	double halfex = 0.0, halfey = 0.0, halfez = 0.0;
	double qa, qb, qc;

	// Auxiliary variables to avoid repeated arithmetic
	q0q0 = q0 * q0;
	q0q1 = q0 * q1;
	q0q2 = q0 * q2;
	q0q3 = q0 * q3;
	q1q1 = q1 * q1;
	q1q2 = q1 * q2;
	q1q3 = q1 * q3;
	q2q2 = q2 * q2;
	q2q3 = q2 * q3;
	q3q3 = q3 * q3;

	// Use magnetometer measurement only when valid (avoids NaN in magnetometer normalisation)
	if((mx != 0.0) && (my != 0.0) && (mz != 0.0)) {
		double hx, hy, bx, bz;
		double halfwx, halfwy, halfwz;

		// Normalise magnetometer measurement
		recipNorm = invSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;

		// Reference direction of Earth's magnetic field
		hx = 2.0 * (mx * (0.5 - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
		hy = 2.0 * (mx * (q1q2 + q0q3) + my * (0.5 - q1q1 - q3q3) + mz * (q2q3 - q0q1));
		bx = sqrt(hx * hx + hy * hy);
		bz = 2.0 * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5 - q1q1 - q2q2));

		// Estimated direction of magnetic field
		halfwx = bx * (0.5 - q2q2 - q3q3) + bz * (q1q3 - q0q2);
		halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
		halfwz = bx * (q0q2 + q1q3) + bz * (0.5 - q1q1 - q2q2);

		// Error is sum of cross product between estimated direction and measured direction of field vectors
		halfex = (my * halfwz - mz * halfwy);
		halfey = (mz * halfwx - mx * halfwz);
		halfez = (mx * halfwy - my * halfwx);
	}

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if((ax != 0.0) && (ay != 0.0) && (az != 0.0)) {
		double halfvx, halfvy, halfvz;

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Estimated direction of gravity
		halfvx = q1q3 - q0q2;
		halfvy = q0q1 + q2q3;
		halfvz = q0q0 - 0.5 + q3q3;

		// Error is sum of cross product between estimated direction and measured direction of field vectors
		halfex += (ay * halfvz - az * halfvy);
		halfey += (az * halfvx - ax * halfvz);
		halfez += (ax * halfvy - ay * halfvx);
	}

	// Apply feedback only when valid data has been gathered from the accelerometer or magnetometer
	if(halfex != 0.0 && halfey != 0.0 && halfez != 0.0) {
		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0) {
			integralFBx += twoKi * halfex * (1.0 / freeIMUsampleFreq);  // integral error scaled by Ki
			integralFBy += twoKi * halfey * (1.0 / freeIMUsampleFreq);
			integralFBz += twoKi * halfez * (1.0 / freeIMUsampleFreq);
			gx += integralFBx;  // apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
		}
		else {
			integralFBx = 0.0; // prevent integral windup
			integralFBy = 0.0;
			integralFBz = 0.0;
		}

		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}

	// Integrate rate of change of quaternion
	gx *= (0.5 * (1.0 / freeIMUsampleFreq));   // pre-multiply common factors
	gy *= (0.5 * (1.0 / freeIMUsampleFreq));
	gz *= (0.5 * (1.0 / freeIMUsampleFreq));
	qa = q0;
	qb = q1;
	qc = q2;
	q0 += (-qb * gx - qc * gy - q3 * gz);
	q1 += (qa * gx + qc * gz - q3 * gy);
	q2 += (qa * gy - qb * gz + q3 * gx);
	q3 += (qa * gz + qb * gy - qc * gx);

	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
}

void getQ(double * q)
{
	double val[9];
	val[0] = accel[0];
	val[1] = accel[1];
	val[2] = accel[2];
	val[3] = gyro[0];
	val[4] = gyro[1];
	val[5] = gyro[2];
	val[6] = magnetom[0];
	val[7] = magnetom[1];
	val[8] = magnetom[2];

	//now = micros();
	//freeIMUsampleFreq = 1.0 / ((now - lastUpdate) / 1000000.0);
	//lastUpdate = now;
	freeIMUsampleFreq = 50.0;

	// gyro values are expressed in deg/sec, the * M_PI/180 will convert it to radians/sec
	//FreeIMUAHRSupdate(val[3] * M_PI/180, val[4] * M_PI/180, val[5] * M_PI/180, val[0], val[1], val[2], val[6], val[7], val[8]);
	FreeIMUAHRSupdate(DEG2RAD(val[3]), DEG2RAD(val[4]), DEG2RAD(val[5]), DEG2RAD(val[0]), DEG2RAD(val[1]), DEG2RAD(val[2]), DEG2RAD(val[6]), DEG2RAD(val[7]), DEG2RAD(val[8]));
	//FreeIMUAHRSupdate(val[3] * M_PI/180, val[4] * M_PI/180, val[5] * M_PI/180, val[0], val[1], val[2], val[7], -val[6], val[8]);

	q[0] = q0;
	q[1] = q1;
	q[2] = q2;
	q[3] = q3;
}

void FreeIMUgetYawPitchRoll(double* y, double* p, double* r)
{
	double q[4]; // quaternion
	double gx, gy, gz; // estimated gravity direction
	getQ(q);

	gx = 2 * (q[1]*q[3] - q[0]*q[2]);
	gy = 2 * (q[0]*q[1] + q[2]*q[3]);
	gz = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];

	*y = atan2(2 * q[1] * q[2] - 2 * q[0] * q[3], 2 * q[0]*q[0] + 2 * q[1] * q[1] - 1) * 180/M_PI;
	*p = atan(gx / sqrt(gy*gy + gz*gz))  * 180/M_PI;
	*r = atan(gy / sqrt(gx*gx + gz*gz))  * 180/M_PI;
}
