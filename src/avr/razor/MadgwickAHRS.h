//=====================================================================================================
// MadgwickAHRS.h
//=====================================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=====================================================================================================
#ifndef MADGWICKAHRS_H_
#define MADGWICKAHRS_H_

extern double madgwick_sampleFreq;  // sample frequency in Hz.
extern double madgwick_beta;  // algorithm gain beta (2 * proportional gain, Kp).

void MadgwickAHRSupdate(double gx, double gy, double gz, double ax, double ay, double az, double mx, double my, double mz);
void MadgwickAHRSupdateIMU(double gx, double gy, double gz, double ax, double ay, double az);
void MadgwickYawPitchRoll(double* y, double* p, double* r);
double invSqrt(double x);

#endif  // MADGWICKAHRS_H_
