/*
 * razor.c
 *
 * Created: 12/09/2012 16:56:36
 * Author: Juan Navarro
 */

#include "razor.h"
#include "razor_math.h"
#include "common.h"
#include <math.h>
#ifndef _MSC_VER
	// If compiling on VisualStudio, compiler is C++ so booleans are implemented.
	// Else, in AtmelStudio, compiler is C so boolean support must be included.
	#include <stdbool.h>
#endif

// DCM parameters
#define Kp_ROLLPITCH 0.02
#define Ki_ROLLPITCH 0.00002
#define Kp_YAW 1.2
#define Ki_YAW 0.00002

// Redefine stdlib's abs() if encountered.
#ifdef abs
	#undef abs
#endif
#define abs(x) ((x) > 0.0 ? (x) : -(x))
#define constrain(x, low, high) ((x) < (low) ? (low) : ((x) > (high) ? (high) : (x)))

// Sensor variables
static double accel[3];  // Actually stores the NEGATED acceleration (equals gravity, if board not moving).
//double accel_min[3];
//double accel_max[3];
//
static double magnetom[3];
//double magnetom_min[3];
//double magnetom_max[3];
//double magnetom_tmp[3];
//
static double gyro[3];
//double gyro_average[3];
//int gyro_num_samples = 0;
//
//// DCM variables
static double MAG_Heading;
static double Accel_Vector[3] = {0.0, 0.0, 0.0}; // Store the acceleration in a vector
static double Gyro_Vector[3] = {0.0, 0.0, 0.0}; // Store the gyros turn rate in a vector
static double Omega_Vector[3] = {0.0, 0.0, 0.0}; // Corrected Gyro_Vector data
static double Omega_P[3] = {0.0, 0.0, 0.0}; // Omega Proportional correction
static double Omega_I[3] = {0.0, 0.0, 0.0}; // Omega Integrator
static double Omega[3] = {0.0, 0.0, 0.0};
static double errorRollPitch[3] = {0.0, 0.0, 0.0};
static double errorYaw[3] = {0.0, 0.0, 0.0};
static double DCM_Matrix[3][3] = {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};
static double Update_Matrix[3][3] = {{0.0, 1.0, 2.0}, {3.0, 4.0, 5.0}, {6.0, 7.0, 8.0}};
static double Temporary_Matrix[3][3] = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
//
//// Euler angles
static double yaw = 0.0;
static double pitch = 0.0;
static double roll = 0.0;
//
//// DCM timing in the main loop
//uint32_t timestamp;
//uint32_t timestamp_old;
static double G_Dt; // Integration time for DCM algorithm
//
//// More output-state variables
//bool output_stream_on;
//bool output_single_on;
//int curr_calibration_sensor = 0;
//bool reset_calibration_session_flag = true;
//int num_accel_errors = 0;
//int num_magn_errors = 0;
//int num_gyro_errors = 0;

static bool g_first_run;


void razor_init(uint8_t dt_ms)
{
	// Initialize global variables.
	for (uint8_t i = 0; i < 3; i++) {
		Accel_Vector[i] = 0.0;
		Gyro_Vector[i] = 0.0;
		Omega_Vector[i] = 0.0;
		Omega_P[i] = 0.0;
		Omega_I[i] = 0.0;
		Omega[i] = 0.0;
		errorRollPitch[i] = 0.0;
		errorYaw[i] = 0.0;
		for (uint8_t j = 0; j < 3; j++) {
			//DCM_Matrix[i][j] = (double)(i == j);
			DCM_Matrix[i][j] = 0.0;
			//Update_Matrix[i][j] = (double)(3 * i + j);
			Update_Matrix[i][j] = 0.0;
			Temporary_Matrix[i][j] = 0.0;
		}
	}
	yaw = pitch = roll = 0.0;
	G_Dt = dt_ms / 1000.0;
	g_first_run = true;
}

// Calculate magnetic heading.
static void Compass_Heading()
{
	double mag_x;
	double mag_y;
	double cos_roll;
	double sin_roll;
	double cos_pitch;
	double sin_pitch;

	cos_roll = cos(roll);
	sin_roll = sin(roll);
	cos_pitch = cos(pitch);
	sin_pitch = sin(pitch);

	// Tilt compensated magnetic field X
	mag_x = magnetom[0] * cos_pitch + magnetom[1] * sin_roll * sin_pitch + magnetom[2] * cos_roll * sin_pitch;
	// Tilt compensated magnetic field Y
	mag_y = magnetom[1] * cos_roll - magnetom[2] * sin_roll;
	// Magnetic Heading
	MAG_Heading = atan2(-mag_y, mag_x);
}

static void Matrix_update(void)
{
#if 1
	// DEV: refactored way. gyro[] was scaled in common.c::sensor_unbias_scale().
	Gyro_Vector[0] = gyro[0]; //gyro x roll
	Gyro_Vector[1] = gyro[1]; //gyro y pitch
	Gyro_Vector[2] = gyro[2]; //gyro z yaw
#else
	// DEV: This is the original Razor way.
	#define DEG2RAD(x) ((x) *  0.01745329252)  // (x) * pi/180
	#define GYRO_GAIN 0.0175 // Same gain on all axes.
	#define GYRO_SCALED_RAD(x) ((x) * DEG2RAD(GYRO_GAIN)) // Calculate the scaled gyro readings in radians per second
	Gyro_Vector[0] = GYRO_SCALED_RAD(gyro[0]); //gyro x roll
	Gyro_Vector[1] = GYRO_SCALED_RAD(gyro[1]); //gyro y pitch
	Gyro_Vector[2] = GYRO_SCALED_RAD(gyro[2]); //gyro z yaw
#endif

	Accel_Vector[0] = accel[0];
	Accel_Vector[1] = accel[1];
	Accel_Vector[2] = accel[2];

	Vector_Add(&Omega[0], &Gyro_Vector[0], &Omega_I[0]);  //adding proportional term
	Vector_Add(&Omega_Vector[0], &Omega[0], &Omega_P[0]); //adding Integrator term

#if DEBUG__NO_DRIFT_CORRECTION // Do not use drift correction
	Update_Matrix[0][0] = 0;
	Update_Matrix[0][1] = -G_Dt * Gyro_Vector[2]; //-z
	Update_Matrix[0][2] =  G_Dt * Gyro_Vector[1]; // y
	Update_Matrix[1][0] =  G_Dt * Gyro_Vector[2]; // z
	Update_Matrix[1][1] = 0;
	Update_Matrix[1][2] = -G_Dt * Gyro_Vector[0];
	Update_Matrix[2][0] = -G_Dt * Gyro_Vector[1];
	Update_Matrix[2][1] =  G_Dt * Gyro_Vector[0];
	Update_Matrix[2][2] = 0;
#else // Use drift correction
	Update_Matrix[0][0] = 0;
	Update_Matrix[0][1] = -G_Dt * Omega_Vector[2]; //-z
	Update_Matrix[0][2] =  G_Dt * Omega_Vector[1]; // y
	Update_Matrix[1][0] =  G_Dt * Omega_Vector[2]; // z
	Update_Matrix[1][1] = 0;
	Update_Matrix[1][2] = -G_Dt * Omega_Vector[0]; //-x
	Update_Matrix[2][0] = -G_Dt * Omega_Vector[1]; //-y
	Update_Matrix[2][1] =  G_Dt * Omega_Vector[0]; // x
	Update_Matrix[2][2] = 0;
#endif

	Matrix_Multiply(DCM_Matrix, Update_Matrix, Temporary_Matrix); //a*b=c

	//Matrix Addition (update)
	for (uint8_t x = 0; x < 3; x++) {
		for (uint8_t y = 0; y < 3; y++) {
			DCM_Matrix[x][y] += Temporary_Matrix[x][y];
		}
	}
}

static void Normalize(void)
{
	double error = 0;
	double temporary[3][3];
	double renorm = 0;

	error = -Vector_Dot_Product(&DCM_Matrix[0][0], &DCM_Matrix[1][0]) * 0.5; //eq.19

	Vector_Scale(&temporary[0][0], &DCM_Matrix[1][0], error); //eq.19
	Vector_Scale(&temporary[1][0], &DCM_Matrix[0][0], error); //eq.19

	Vector_Add(&temporary[0][0], &temporary[0][0], &DCM_Matrix[0][0]); //eq.19
	Vector_Add(&temporary[1][0], &temporary[1][0], &DCM_Matrix[1][0]); //eq.19

	Vector_Cross_Product(&temporary[2][0], &temporary[0][0], &temporary[1][0]); // c= a x b //eq.20

	renorm = 0.5 * (3 - Vector_Dot_Product(&temporary[0][0], &temporary[0][0])); //eq.21
	Vector_Scale(&DCM_Matrix[0][0], &temporary[0][0], renorm);

	renorm = 0.5 * (3 - Vector_Dot_Product(&temporary[1][0], &temporary[1][0])); //eq.21
	Vector_Scale(&DCM_Matrix[1][0], &temporary[1][0], renorm);

	renorm = 0.5 * (3 - Vector_Dot_Product(&temporary[2][0], &temporary[2][0])); //eq.21
	Vector_Scale(&DCM_Matrix[2][0], &temporary[2][0], renorm);
}

static void Drift_correction(void)
{
	double mag_heading_x;
	double mag_heading_y;
	double errorCourse;
	//Compensation the Roll, Pitch and Yaw drift.
	static double Scaled_Omega_P[3];
	static double Scaled_Omega_I[3];
	double Accel_magnitude;
	double Accel_weight;


	//*****Roll and Pitch***************

	// Calculate the magnitude of the accelerometer vector
	Accel_magnitude = sqrt(Accel_Vector[0]*Accel_Vector[0] + Accel_Vector[1]*Accel_Vector[1] + Accel_Vector[2]*Accel_Vector[2]);
	Accel_magnitude = Accel_magnitude / ACC_SENSITIVITY; // Scale to gravity.
	// Dynamic weighting of accelerometer info (reliability filter)
	// Weight for accelerometer info (<0.5G = 0.0, 1G = 1.0 , >1.5G = 0.0)
	Accel_weight = constrain(1.0 - 2.0 * abs(1.0 - Accel_magnitude), 0.0, 1.0);

	Vector_Cross_Product(&errorRollPitch[0], &Accel_Vector[0], &DCM_Matrix[2][0]); //adjust the ground of reference
	Vector_Scale(&Omega_P[0], &errorRollPitch[0], Kp_ROLLPITCH*Accel_weight);

	Vector_Scale(&Scaled_Omega_I[0], &errorRollPitch[0], Ki_ROLLPITCH*Accel_weight);
	Vector_Add(Omega_I, Omega_I, Scaled_Omega_I);

	//*****YAW***************
	// We make the gyro YAW drift correction based on compass magnetic heading

	mag_heading_x = cos(MAG_Heading);
	mag_heading_y = sin(MAG_Heading);
	errorCourse = (DCM_Matrix[0][0] * mag_heading_y) - (DCM_Matrix[1][0] * mag_heading_x);  //Calculating YAW error
	Vector_Scale(errorYaw, &DCM_Matrix[2][0], errorCourse); //Applies the yaw correction to the XYZ rotation of the aircraft, depending on the position.

	Vector_Scale(&Scaled_Omega_P[0], &errorYaw[0], Kp_YAW); //.01 proportional of YAW.
	Vector_Add(Omega_P, Omega_P, Scaled_Omega_P); //Adding Proportional.

	Vector_Scale(&Scaled_Omega_I[0], &errorYaw[0], Ki_YAW); //.00001 Integrator
	Vector_Add(Omega_I, Omega_I, Scaled_Omega_I); //adding integrator to the Omega_I
}

static void Euler_angles(void)
{
	yaw   = atan2(DCM_Matrix[1][0], DCM_Matrix[0][0]);
	pitch = -asin(DCM_Matrix[2][0]);
	roll  = atan2(DCM_Matrix[2][1], DCM_Matrix[2][2]);
}

void razor_heading(double out_YawPitchRoll_rad[3], /*const*/ double in_AccGyroMag_calib[3][3])
{
	accel[0] = in_AccGyroMag_calib[0][0];
	accel[1] = in_AccGyroMag_calib[0][1];
	accel[2] = in_AccGyroMag_calib[0][2];
	gyro[0] = in_AccGyroMag_calib[1][0];
	gyro[1] = in_AccGyroMag_calib[1][1];
	gyro[2] = in_AccGyroMag_calib[1][2];
	magnetom[0] = in_AccGyroMag_calib[2][0];
	magnetom[1] = in_AccGyroMag_calib[2][1];
	magnetom[2] = in_AccGyroMag_calib[2][2];

	if (g_first_run) {
		g_first_run = false;

		// Init DCM with unfiltered orientation.

		double temp1[3];
		double temp2[3];
		double xAxis[3] = {1.0, 0.0, 0.0};

		// GET PITCH
		// Using y-z-plane-component/x-component of gravity vector
		pitch = -atan2(accel[0], sqrt(accel[1] * accel[1] + accel[2] * accel[2]));

		// GET ROLL
		// Compensate pitch of gravity vector
		Vector_Cross_Product(temp1, accel, xAxis);
		Vector_Cross_Product(temp2, xAxis, temp1);
		// Normally using x-z-plane-component/y-component of compensated gravity vector
		// roll = atan2(temp2[1], sqrt(temp2[0] * temp2[0] + temp2[2] * temp2[2]));
		// Since we compensated for pitch, x-z-plane-component equals z-component:
		roll = atan2(temp2[1], temp2[2]);

		// GET YAW
		Compass_Heading();
		yaw = MAG_Heading;

		// Init rotation matrix
		init_rotation_matrix(DCM_Matrix, yaw, pitch, roll);
	}
	else {
		// Run DCM algorithm.
		Compass_Heading();
		Matrix_update();
		Normalize();
		Drift_correction();
		Euler_angles();
	}

	out_YawPitchRoll_rad[0] = yaw;
	out_YawPitchRoll_rad[1] = pitch;
	out_YawPitchRoll_rad[2] = roll;
}
