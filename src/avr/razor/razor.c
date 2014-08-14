#include "razor.h"
#include "../board/sensor/sensor.h"
#include "MadgwickAHRS.h"
#include "FreeIMU.h"
#include <stdbool.h>
#include <math.h>
#include <string.h>  // memcpy().

#define MS2S(x) ((x) * 0.001)  // Milliseconds to seconds.


// Select your startup output mode and format here!
//output_mode_t output_mode = STARTUP_OUTPUT_MODE;
//bool output_serialchart = true;
//output_format_t output_format = STARTUP_OUTPUT_FORMAT;

// If set true, an error message will be output if we fail to read sensor data.
// Message format: "!ERR: reading <sensor>", followed by "\r\n".
char output_errors = 0;  // true or false


/*
CAMBIOS

menu: cambio '#' por 'q'.
*/

/***************************************************************************************************************
* Razor AHRS Firmware v1.4.1
* 9 Degree of Measurement Attitude and Heading Reference System
* for Sparkfun "9DOF Razor IMU" (SEN-10125 and SEN-10736)
* and "9DOF Sensor Stick" (SEN-10183, 10321 and SEN-10724)
*
* Released under GNU GPL (General Public License) v3.0
* Copyright (C) 2011-2012 Quality & Usability Lab, Deutsche Telekom Laboratories, TU Berlin
*
* Infos, updates, bug reports and feedback:
*     http://dev.qu.tu-berlin.de/projects/sf-razor-9dof-ahrs
*
*
* History:
*   * Original code (http://code.google.com/p/sf9domahrs/) by Doug Weibel and Jose Julio,
*     based on ArduIMU v1.5 by Jordi Munoz and William Premerlani, Jose Julio and Doug Weibel. Thank you!
*
*   * Updated code (http://groups.google.com/group/sf_9dof_ahrs_update) by David Malik (david.zsolt.malik@gmail.com)
*     for new Sparkfun 9DOF Razor hardware (SEN-10125).
*
*   * Updated and extended by Peter Bartz (peter-bartz@gmx.de):
*     * v1.3.0
*       * Cleaned up, streamlined and restructured most of the code to make it more comprehensible.
*       * Added sensor calibration (improves precision and responsiveness a lot!).
*       * Added binary yaw/pitch/roll output.
*       * Added basic serial command interface to set output modes/calibrate sensors/synch stream/etc.
*       * Added support to synch automatically when using Rovering Networks Bluetooth modules (and compatible).
*       * Wrote new easier to use test program (using Processing).
*       * Added support for new version of "9DOF Razor IMU": SEN-10736.
*       --> The output of this code is not compatible with the older versions!
*       --> A Processing sketch to test the tracker is available.
*     * v1.3.1
*       * Initializing rotation matrix based on start-up sensor readings -> orientation OK right away.
*       * Adjusted gyro low-pass filter and output rate settings.
*     * v1.3.2
*       * Adapted code to work with new Arduino 1.0 (and older versions still).
*     * v1.3.3
*       * Improved synching.
*     * v1.4.0
*       * Added support for SparkFun "9DOF Sensor Stick" (versions SEN-10183, SEN-10321 and SEN-10724).
*     * v1.4.1
*       * Added output modes to read raw and/or calibrated sensor data in text or binary format.
*       * Added static magnetometer soft iron distortion compensation
*
* TODOs:
*   * Allow optional use of EEPROM for storing and reading calibration values.
*   * Use self-test and temperature-compensation features of the sensors.
***************************************************************************************************************/

/*
  Axis definition (differs from definition printed on the board!):
    X axis pointing forward (towards the short edge with the connector holes)
    Y axis pointing to the right
    and Z axis pointing down.

  Positive yaw   : clockwise
  Positive roll  : right wing down
  Positive pitch : nose up

  Transformation order: first yaw then pitch then roll.
*/

/*
  Serial commands that the firmware understands:

  "#o<params>" - Set OUTPUT mode and parameters. The available options are:

      // Streaming output
      "#o0" - DISABLE continuous streaming output. Also see #f below.
      "#o1" - ENABLE continuous streaming output.

      // Angles output
      "#ob" - Output angles in BINARY format (yaw/pitch/roll as binary float, so one output frame
              is 3x4 = 12 bytes long).
      "#ot" - Output angles in TEXT format (Output frames have form like "#YPR=-142.28,-5.38,33.52",
              followed by carriage return and line feed [\r\n]).

      // Sensor calibration
      "#oc" - Go to CALIBRATION output mode.
      "#on" - When in calibration mode, go on to calibrate NEXT sensor.

      // Sensor data output
      "#osct" - Output CALIBRATED SENSOR data of all 9 axes in TEXT format.
                One frame consist of three lines - one for each sensor: acc, mag, gyr.
      "#osrt" - Output RAW SENSOR data of all 9 axes in TEXT format.
                One frame consist of three lines - one for each sensor: acc, mag, gyr.
      "#osbt" - Output BOTH raw and calibrated SENSOR data of all 9 axes in TEXT format.
                One frame consist of six lines - like #osrt and #osct combined (first RAW, then CALIBRATED).
                NOTE: This is a lot of number-to-text conversion work for the little 8MHz chip on the Razor boards.
                In fact it's too much and an output frame rate of 50Hz can not be maintained. #osbb.
      "#oscb" - Output CALIBRATED SENSOR data of all 9 axes in BINARY format.
                One frame consist of three 3x3 float values = 36 bytes. Order is: acc x/y/z, mag x/y/z, gyr x/y/z.
      "#osrb" - Output RAW SENSOR data of all 9 axes in BINARY format.
                One frame consist of three 3x3 float values = 36 bytes. Order is: acc x/y/z, mag x/y/z, gyr x/y/z.
      "#osbb" - Output BOTH raw and calibrated SENSOR data of all 9 axes in BINARY format.
                One frame consist of 2x36 = 72 bytes - like #osrb and #oscb combined (first RAW, then CALIBRATED).

      // Error message output
      "#oe0" - Disable ERROR message output.
      "#oe1" - Enable ERROR message output.


  "#f" - Request one output frame - useful when continuous output is disabled and updates are
         required in larger intervals only. Though #f only requests one reply, replies are still
         bound to the internal 20ms (50Hz) time raster. So worst case delay that #f can add is 19.99ms.


  "#s<xy>" - Request synch token - useful to find out where the frame boundaries are in a continuous
         binary stream or to see if tracker is present and answering. The tracker will send
         "#SYNCH<xy>\r\n" in response (so it's possible to read using a readLine() function).
         x and y are two mandatory but arbitrary bytes that can be used to find out which request
         the answer belongs to.


  ("#C" and "#D" - Reserved for communication with optional Bluetooth module.)

  Newline characters are not required. So you could send "#ob#o1#s", which
  would set binary output mode, enable continuous streaming output and request
  a synch token all at once.

  The status LED will be on if streaming output is enabled and off otherwise.

  Byte order of binary output is little-endian: least significant byte comes first.
*/



// ------------------------
// Android filter
// angular speeds from gyro
//double gyro[3];
// rotation matrix from gyro data
double gyroMatrix[9];
// orientation angles from gyro matrix
double gyroOrientation[3];
// magnetic field vector
//double magnet[3];
// accelerometer vector
//double accel[3];
// orientation angles from accel and magnet
double accMagOrientation[3];
// final orientation angles from sensor fusion (radians).
double fusedOrientation[3];
// accelerometer and magnetometer based rotation matrix
double rotationMatrix[9];

// #define TIME_CONSTANT 30 <-- EN razor_config.h, OUTPUT__DATA_INTERVAL
//#define FILTER_COEFFICIENT 0.98
#define FILTER_COEFFICIENT 0.95
//Timer fuseTimer = new Timer();
// ------------------------


// Reset calibration session if reset_calibration_session_flag is set
void check_reset_calibration_session()
{
  // Raw sensor values have to be read already, but no error compensation applied

  // Reset this calibration session?
  if (!reset_calibration_session_flag) return;

  // Reset acc and mag calibration variables
  for (int i = 0; i < 3; i++) {
    accel_min[i] = accel_max[i] = accel[i];
    magnetom_min[i] = magnetom_max[i] = magnetom[i];
  }

  // Reset gyro calibration variables
  gyro_num_samples = 0;  // Reset gyro calibration averaging
  gyro_average[0] = gyro_average[1] = gyro_average[2] = 0.0f;

  reset_calibration_session_flag = false;
}




// Blocks until another byte is available on serial port
//char readChar()
//{
	//while (serial_available() < 1) { } // Block
	//return serial_read();
//}



//// Process incoming control messages.
//void razor_loop_input(void)
//{
	//if (serial_available() < 1)  // 1, so we can pause with just one key press.
		//return;
//
	//if (serial_read() != 'q')
		//return;
//
	//int command = serial_read();
	//switch (command)
	//{
		//case 'o':
		//// Set _o_utput mode
		//{
			//char output_param = readChar();
			//switch (output_param)
			//{
				//case 'c':
				//// Go to _c_alibration mode
				//{
					//output_mode = OUTPUT__MODE_CALIBRATE_SENSORS;
					//reset_calibration_session_flag = true;
					//break;
				//}
				//case 'n':
				//{
					//// Calibrate _n_ext sensor
					//curr_calibration_sensor = (curr_calibration_sensor + 1) % 3;
					//reset_calibration_session_flag = true;
					//break;
				//}
				//case 's':
				//case 'S':
				//{
					//output_serialchart = (output_param == 'S');
					//char values_param = readChar();
					//char format_param = readChar();
					//switch (values_param)
					//{
						//case 'r':
						//{
							//// Output _r_aw sensor values
							//output_mode = OUTPUT__MODE_SENSORS_RAW;
							//break;
						//}
						//case 'c':
						//{
							//// Output _c_alibrated sensor values
							//output_mode = OUTPUT__MODE_SENSORS_CALIB;
							//break;
						//}
						//case 'b':
						//{
							//// Output _b_oth sensor values (raw and calibrated)
							//output_mode = OUTPUT__MODE_SENSORS_BOTH;
							//break;
						//}
						//// else skip unknown character.
					//}
					//switch (format_param)
					//{
						//case 't':
						//{
							//// Output values as _t_text
							//output_format = OUTPUT__FORMAT_TEXT;
							//break;
						//}
						//case 'b':
						//{
							//// Output values in _b_inary format
							//output_format = OUTPUT__FORMAT_BINARY;
							//break;
						//}
						//// else skip unknown character.
					//}
					//break;
				//}
				//case 'b':
				//{
					//// Output angles in _b_inary format
					////output_mode = OUTPUT__MODE_YPR_RAZOR;
					////output_mode = OUTPUT__MODE_YPR_MADGWICK;
					////output_mode = STARTUP_OUTPUT_MODE;
//
					//// Do not change the output mode.
					//// (Change manually through serial Terminal).
					//output_format = OUTPUT__FORMAT_BINARY;
					//break;
				//}
				//case 't':
				//case 'T':
				//{
					//output_serialchart = (output_param == 'T');
					//output_mode = OUTPUT__MODE_YPR_RAZOR;
					//output_format = OUTPUT__FORMAT_TEXT;
					//break;
				//}
				//case 'm':
				//case 'M':
				//{
					//output_serialchart = (output_param == 'M');
					//output_mode = OUTPUT__MODE_YPR_MADGWICK;
					//output_format = OUTPUT__FORMAT_TEXT;
					//break;
				//}
				//case 'f':
				//case 'F':
				//{
					//output_serialchart = (output_param == 'F');
					//output_mode = OUTPUT__MODE_YPR_FREEIMU;
					//output_format = OUTPUT__FORMAT_TEXT;
					//break;
				//}
				//case '0':
				//{
					//// Disable continuous streaming output
					//turn_output_stream_off();
					//reset_calibration_session_flag = true;
					//break;
				//}
				//case '1':
				//{
					//// Enable continuous streaming output
					//reset_calibration_session_flag = true;
					//turn_output_stream_on();
					//break;
				//}
				//case 'e':
				//// _e_rror output settings
				//{
					//char error_param = readChar();
					//if (error_param == '0') output_errors = false;
					//else if (error_param == '1') output_errors = true;
					//else if (error_param == 'c') // get error count
						//printf("#AMG-ERR: %d, %d, %d\r\n", num_accel_errors, num_magn_errors, num_gyro_errors);
					//break;
				//}
				//// else skip unknown character.
			//}
			//break;
		//}
		//case 'f':
		//// request one output _f_rame
		//{
			//output_single_on = true;
			//break;
		//}
		//case 's':
		//// _s_ynch request
		//{
			//// Read ID
			//uint8_t id[2];
			//id[0] = readChar();
			//id[1] = readChar();
//
			//// Reply with synch message
			//printf("#SYNCH");
			//serial_write(id, 2);
			//printf("\r\n");
			//break;
		//}
		//case 'r':
		////J Force reset of internal algorithm.
		//{
			//razor_setup();
			//break;
		//}
		//case 'c':
		////J Force check for MEM sensors.
		//{
			//sensor_init();
			//break;
		//}
		//case 'b':
		////J Launch the embedded STK500v2 bootloader.
		//{
			//stk500v2_bootloader();
			//break;
		//}
		//// else skip unknown character.
	//}
//}

/**
* <p>
* Computes the inclination matrix <b>I</b> as well as the rotation matrix
* <b>R</b> transforming a vector from the device coordinate system to the
* world's coordinate system which is defined as a direct orthonormal basis,
* where:
* </p>
*
* <ul>
* <li>X is defined as the vector product <b>Y.Z</b> (It is tangential to
* the ground at the device's current location and roughly points East).</li>
* <li>Y is tangential to the ground at the device's current location and
* points towards the magnetic North Pole.</li>
* <li>Z points towards the sky and is perpendicular to the ground.</li>
* </ul>
*
* <p>
* <center><img src="../../../images/axis_globe.png"
* alt="World coordinate-system diagram." border="0" /></center>
* </p>
*
* <p>
* <hr>
* <p>
* By definition:
* <p>
* [0 0 g] = <b>R</b> * <b>gravity</b> (g = magnitude of gravity)
* <p>
* [0 m 0] = <b>I</b> * <b>R</b> * <b>geomagnetic</b> (m = magnitude of
* geomagnetic field)
* <p>
* <b>R</b> is the identity matrix when the device is aligned with the
* world's coordinate system, that is, when the device's X axis points
* toward East, the Y axis points to the North Pole and the device is facing
* the sky.
*
* <p>
* <b>I</b> is a rotation matrix transforming the geomagnetic vector into
* the same coordinate space as gravity (the world's coordinate space).
* <b>I</b> is a simple rotation around the X axis. The inclination angle in
* radians can be computed with {@link #getInclination}.
* <hr>
*
* <p>
* Each matrix is returned either as a 3x3 or 4x4 row-major matrix depending
* on the length of the passed array:
* <p>
* <u>If the array length is 16:</u>
*
* <pre>
*   /  M[ 0]   M[ 1]   M[ 2]   M[ 3]  \
*   |  M[ 4]   M[ 5]   M[ 6]   M[ 7]  |
*   |  M[ 8]   M[ 9]   M[10]   M[11]  |
*   \  M[12]   M[13]   M[14]   M[15]  /
*</pre>
*
* This matrix is ready to be used by OpenGL ES's
* {@link javax.microedition.khronos.opengles.GL10#glLoadMatrixf(float[], int)
* glLoadMatrixf(float[], int)}.
* <p>
* Note that because OpenGL matrices are column-major matrices you must
* transpose the matrix before using it. However, since the matrix is a
* rotation matrix, its transpose is also its inverse, conveniently, it is
* often the inverse of the rotation that is needed for rendering; it can
* therefore be used with OpenGL ES directly.
* <p>
* Also note that the returned matrices always have this form:
*
* <pre>
*   /  M[ 0]   M[ 1]   M[ 2]   0  \
*   |  M[ 4]   M[ 5]   M[ 6]   0  |
*   |  M[ 8]   M[ 9]   M[10]   0  |
*   \      0       0       0   1  /
*</pre>
*
* <p>
* <u>If the array length is 9:</u>
*
* <pre>
*   /  M[ 0]   M[ 1]   M[ 2]  \
*   |  M[ 3]   M[ 4]   M[ 5]  |
*   \  M[ 6]   M[ 7]   M[ 8]  /
*</pre>
*
* <hr>
* <p>
* The inverse of each matrix can be computed easily by taking its
* transpose.
*
* <p>
* The matrices returned by this function are meaningful only when the
* device is not free-falling and it is not close to the magnetic north. If
* the device is accelerating, or placed into a strong magnetic field, the
* returned matrices may be inaccurate.
*
* @param R
*        is an array of 9 floats holding the rotation matrix <b>R</b> when
*        this function returns. R can be null.
*        <p>
*
* @param I
*        is an array of 9 floats holding the rotation matrix <b>I</b> when
*        this function returns. I can be null.
*        <p>
*
* @param gravity
*        is an array of 3 floats containing the gravity vector expressed in
*        the device's coordinate. You can simply use the
*        {@link android.hardware.SensorEvent#values values} returned by a
*        {@link android.hardware.SensorEvent SensorEvent} of a
*        {@link android.hardware.Sensor Sensor} of type
*        {@link android.hardware.Sensor#TYPE_ACCELEROMETER
*        TYPE_ACCELEROMETER}.
*        <p>
*
* @param geomagnetic
*        is an array of 3 floats containing the geomagnetic vector
*        expressed in the device's coordinate. You can simply use the
*        {@link android.hardware.SensorEvent#values values} returned by a
*        {@link android.hardware.SensorEvent SensorEvent} of a
*        {@link android.hardware.Sensor Sensor} of type
*        {@link android.hardware.Sensor#TYPE_MAGNETIC_FIELD
*        TYPE_MAGNETIC_FIELD}.
*
* @return <code>true</code> on success, <code>false</code> on failure (for
*         instance, if the device is in free fall). On failure the output
*         matrices are not modified.
*
* @see #getInclination(float[])
* @see #getOrientation(float[], float[])
* @see #remapCoordinateSystem(float[], int, int, float[])
*/
// Android SensorManager.getRotationMatrix().
// Source:
// http://grepcode.com/file_/repository.grepcode.com/java/ext/com.google.android/android/4.1.1_r1/android/hardware/SensorManager.java/?v=source
bool getRotationMatrix(double R[9], double I[9], double gravity[3], double geomagnetic[3])
{
	// TODO: move this to native code for efficiency
	double Ax = gravity[0];
	double Ay = gravity[1];
	double Az = gravity[2];
	const double Ex = geomagnetic[0];
	const double Ey = geomagnetic[1];
	const double Ez = geomagnetic[2];
	double Hx = Ey*Az - Ez*Ay;
	double Hy = Ez*Ax - Ex*Az;
	double Hz = Ex*Ay - Ey*Ax;
	const double normH = sqrt(Hx*Hx + Hy*Hy + Hz*Hz);
	if (normH < 0.1f) {
		// device is close to free fall (or in space?), or close to
		// magnetic north pole. Typical values are > 100.
		return false;
	}
	const double invH = 1.0f / normH;
	Hx *= invH;
	Hy *= invH;
	Hz *= invH;
	const double invA = 1.0f / sqrt(Ax*Ax + Ay*Ay + Az*Az);
	Ax *= invA;
	Ay *= invA;
	Az *= invA;
	const double Mx = Ay*Hz - Az*Hy;
	const double My = Az*Hx - Ax*Hz;
	const double Mz = Ax*Hy - Ay*Hx;
	if (R != NULL) {
			R[0] = Hx;     R[1] = Hy;     R[2] = Hz;
			R[3] = Mx;     R[4] = My;     R[5] = Mz;
			R[6] = Ax;     R[7] = Ay;     R[8] = Az;
	}
	if (I != NULL) {
		// compute the inclination matrix by projecting the geomagnetic
		// vector onto the Z (gravity) and X (horizontal component
		// of geomagnetic vector) axes.
		const double invE = 1.0f / sqrt(Ex*Ex + Ey*Ey + Ez*Ez);
		const double c = (Ex*Mx + Ey*My + Ez*Mz) * invE;
		const double s = (Ex*Ax + Ey*Ay + Ez*Az) * invE;
			I[0] = 1;     I[1] = 0;     I[2] = 0;
			I[3] = 0;     I[4] = c;     I[5] = s;
			I[6] = 0;     I[7] =-s;     I[8] = c;
	}
	return true;
}

/**
* Computes the device's orientation based on the rotation matrix.
* <p>
* When it returns, the array values is filled with the result:
* <ul>
* <li>values[0]: <i>yaw</i>, rotation around the Z axis.</li>
* <li>values[1]: <i>pitch</i>, rotation around the X axis.</li>
* <li>values[2]: <i>roll</i>, rotation around the Y axis.</li>
* </ul>
* <p>The reference coordinate-system used is different from the world
* coordinate-system defined for the rotation matrix:</p>
* <ul>
* <li>X is defined as the vector product <b>Y.Z</b> (It is tangential to
* the ground at the device's current location and roughly points West).</li>
* <li>Y is tangential to the ground at the device's current location and
* points towards the magnetic North Pole.</li>
* <li>Z points towards the center of the Earth and is perpendicular to the ground.</li>
* </ul>
*
* <p>
* <center><img src="../../../images/axis_globe_inverted.png"
* alt="Inverted world coordinate-system diagram." border="0" /></center>
* </p>
* <p>
* All three angles above are in <b>radians</b> and <b>positive</b> in the
* <b>counter-clockwise</b> direction.
*
* @param R
*        rotation matrix see {@link #getRotationMatrix}.
*
* @param values
*        an array of 3 floats to hold the result.
*
* @return The array values passed as argument.
*
* @see #getRotationMatrix(float[], float[], float[], float[])
* @see GeomagneticField
*/
// Android: SensorManager.getOrientation().
void getOrientation(double out[3], double R[9])
{
	out[0] = atan2(R[1], R[4]);
	out[1] = asin(-R[7]);
	out[2] = atan2(-R[6], R[8]);
}

void calculateAccMagOrientation(void)
{
	if (getRotationMatrix(rotationMatrix, NULL, accel, magnetom)) {
		getOrientation(accMagOrientation, rotationMatrix);
	}
	else {
		printf("NO CALCULO! ARGH!\r\n");
	}
}

// This function is borrowed from the Android reference
// at http://developer.android.com/reference/android/hardware/SensorEvent.html#values
// It calculates a rotation vector from the gyroscope angular speed values.
void getRotationVectorFromGyro(double deltaRotationVector[4], double dT)
{
	static const double EPSILON = 0.000000001f;
	double normValues[3] = {0.0, 0.0, 0.0};

	// Calculate the angular speed of the sample (squared).
	double omegaMagnitude_sq = gyro[0] * gyro[0] + gyro[1] * gyro[1] + gyro[2] * gyro[2];

	// Normalize the rotation vector if it's big enough to get the axis
	if (omegaMagnitude_sq > EPSILON) {
		double recipNorm = invSqrt(omegaMagnitude_sq);
		normValues[0] = gyro[0] * recipNorm;
		normValues[1] = gyro[1] * recipNorm;
		normValues[2] = gyro[2] * recipNorm;
	}

	// Integrate around this axis with the angular speed by the timestep
	// in order to get a delta rotation from this sample over the timestep
	// We will convert this axis-angle representation of the delta rotation
	// into a quaternion before turning it into the rotation matrix.
	double thetaOverTwo = sqrt(omegaMagnitude_sq) * dT / 2.0f;
	double sinThetaOverTwo = sin(thetaOverTwo);
	double cosThetaOverTwo = cos(thetaOverTwo);
	deltaRotationVector[0] = sinThetaOverTwo * normValues[0];
	deltaRotationVector[1] = sinThetaOverTwo * normValues[1];
	deltaRotationVector[2] = sinThetaOverTwo * normValues[2];
	deltaRotationVector[3] = cosThetaOverTwo;
}

void matrixMultiplication(double out[9], double A[9], double B[9])
{
	double tmp[9];

	tmp[0] = A[0] * B[0] + A[1] * B[3] + A[2] * B[6];
	tmp[1] = A[0] * B[1] + A[1] * B[4] + A[2] * B[7];
	tmp[2] = A[0] * B[2] + A[1] * B[5] + A[2] * B[8];

	tmp[3] = A[3] * B[0] + A[4] * B[3] + A[5] * B[6];
	tmp[4] = A[3] * B[1] + A[4] * B[4] + A[5] * B[7];
	tmp[5] = A[3] * B[2] + A[4] * B[5] + A[5] * B[8];

	tmp[6] = A[6] * B[0] + A[7] * B[3] + A[8] * B[6];
	tmp[7] = A[6] * B[1] + A[7] * B[4] + A[8] * B[7];
	tmp[8] = A[6] * B[2] + A[7] * B[5] + A[8] * B[8];

	memcpy(out, tmp, sizeof(out));
}

void getRotationMatrixFromOrientation(double out[9], double orientation[3])
{
	double xM[9];
	double yM[9];
	double zM[9];

	double sinX = sin(orientation[1]);
	double cosX = cos(orientation[1]);
	double sinY = sin(orientation[2]);
	double cosY = cos(orientation[2]);
	double sinZ = sin(orientation[0]);
	double cosZ = cos(orientation[0]);

	// rotation about x-axis (pitch)
	xM[0] = 1.0f; xM[1] = 0.0f; xM[2] = 0.0f;
	xM[3] = 0.0f; xM[4] = cosX; xM[5] = sinX;
	xM[6] = 0.0f; xM[7] = -sinX; xM[8] = cosX;

	// rotation about y-axis (roll)
	yM[0] = cosY; yM[1] = 0.0f; yM[2] = sinY;
	yM[3] = 0.0f; yM[4] = 1.0f; yM[5] = 0.0f;
	yM[6] = -sinY; yM[7] = 0.0f; yM[8] = cosY;

	// rotation about z-axis (azimuth)
	zM[0] = cosZ; zM[1] = sinZ; zM[2] = 0.0f;
	zM[3] = -sinZ; zM[4] = cosZ; zM[5] = 0.0f;
	zM[6] = 0.0f; zM[7] = 0.0f; zM[8] = 1.0f;

	// rotation order is y, x, z (roll, pitch, azimuth)
	matrixMultiplication(out, xM, yM);
	matrixMultiplication(out, zM, out);
}

/** Helper function to convert a rotation vector to a rotation matrix.
*  Given a rotation vector (presumably from a ROTATION_VECTOR sensor), returns a
*  9  or 16 element rotation matrix in the array R.  R must have length 9 or 16.
*  If R.length == 9, the following matrix is returned:
* <pre>
*   /  R[ 0]   R[ 1]   R[ 2]   \
*   |  R[ 3]   R[ 4]   R[ 5]   |
*   \  R[ 6]   R[ 7]   R[ 8]   /
*</pre>
* If R.length == 16, the following matrix is returned:
* <pre>
*   /  R[ 0]   R[ 1]   R[ 2]   0  \
*   |  R[ 4]   R[ 5]   R[ 6]   0  |
*   |  R[ 8]   R[ 9]   R[10]   0  |
*   \  0       0       0       1  /
*</pre>
*  @param rotationVector the rotation vector to convert
*  @param R an array of floats in which to store the rotation matrix
*/
// Android SensorManager.getRotationMatrixFromVector().
void getRotationMatrixFromVector(double out[9], double rotationVector[4])
{
    double q0 = rotationVector[3];
    double q1 = rotationVector[0];
    double q2 = rotationVector[1];
    double q3 = rotationVector[2];

    double sq_q1 = 2 * q1 * q1;
    double sq_q2 = 2 * q2 * q2;
    double sq_q3 = 2 * q3 * q3;
    double q1_q2 = 2 * q1 * q2;
    double q3_q0 = 2 * q3 * q0;
    double q1_q3 = 2 * q1 * q3;
    double q2_q0 = 2 * q2 * q0;
    double q2_q3 = 2 * q2 * q3;
    double q1_q0 = 2 * q1 * q0;

    out[0] = 1 - sq_q2 - sq_q3;
    out[1] = q1_q2 - q3_q0;
    out[2] = q1_q3 + q2_q0;

    out[3] = q1_q2 + q3_q0;
    out[4] = 1 - sq_q1 - sq_q3;
    out[5] = q2_q3 - q1_q0;

    out[6] = q1_q3 - q2_q0;
    out[7] = q2_q3 + q1_q0;
    out[8] = 1 - sq_q1 - sq_q2;
}

// This function performs the integration of the gyroscope data.
// It writes the gyroscope based orientation into gyroOrientation.
void gyroFunction(void)
{
	//static double NS2S = 1.0f / 1000000000.0f;
	//double timestamp;

	// don't start until first accelerometer/magnetometer orientation has been acquired
	//if (accMagOrientation == null)
	//return;

	// initialisation of the gyroscope based rotation matrix
	static bool initState = true;
	if (initState) {
		double initMatrix[9];
		getRotationMatrixFromOrientation(initMatrix, accMagOrientation);
		double test[3];
		getOrientation(test, initMatrix);
		matrixMultiplication(gyroMatrix, gyroMatrix, initMatrix);
		initState = false;
	}

	// copy the new gyro values into the gyro array
	// convert the raw gyro data into a rotation vector
	double deltaVector[4];
		//const double dT = (timer_systime() - timestamp) * NS2S;
		//System.arraycopy(event.values, 0, gyro, 0, 3);
		getRotationVectorFromGyro(deltaVector, G_Dt);

	// measurement done, save current time for next interval
	//timestamp = timer_systime();

	// convert rotation vector into rotation matrix
	double deltaMatrix[9];
	getRotationMatrixFromVector(deltaMatrix, deltaVector);

	// apply the new rotation interval on the gyroscope based rotation matrix
	matrixMultiplication(gyroMatrix, gyroMatrix, deltaMatrix);

	// get the gyroscope based orientation from the rotation matrix
	getOrientation(gyroMatrix, gyroOrientation);
}

void calculateFusedOrientation(void)
{
	double oneMinusCoeff = 1.0f - FILTER_COEFFICIENT;

	/*
		* Fix for 179° <--> -179° transition problem:
		* Check whether one of the two orientation angles (gyro or accMag) is negative while the other one is positive.
		* If so, add 360° (2 * math.PI) to the negative value, perform the sensor fusion, and remove the 360° from the result
		* if it is greater than 180°. This stabilizes the output in positive-to-negative-transition cases.
		*/

	// yaw
	if (gyroOrientation[0] < -0.5 * M_PI && accMagOrientation[0] > 0.0) {
		fusedOrientation[0] = (double) (FILTER_COEFFICIENT * (gyroOrientation[0] + 2.0 * M_PI) + oneMinusCoeff * accMagOrientation[0]);
		fusedOrientation[0] -= (fusedOrientation[0] > M_PI) ? 2.0 * M_PI : 0;
	}
	else if (accMagOrientation[0] < -0.5 * M_PI && gyroOrientation[0] > 0.0) {
		fusedOrientation[0] = (double) (FILTER_COEFFICIENT * gyroOrientation[0] + oneMinusCoeff * (accMagOrientation[0] + 2.0 * M_PI));
		fusedOrientation[0] -= (fusedOrientation[0] > M_PI)? 2.0 * M_PI : 0;
	}
	else {
		fusedOrientation[0] = FILTER_COEFFICIENT * gyroOrientation[0] + oneMinusCoeff * accMagOrientation[0];
	}

	// pitch
	if (gyroOrientation[1] < -0.5 * M_PI && accMagOrientation[1] > 0.0) {
		fusedOrientation[1] = (double) (FILTER_COEFFICIENT * (gyroOrientation[1] + 2.0 * M_PI) + oneMinusCoeff * accMagOrientation[1]);
		fusedOrientation[1] -= (fusedOrientation[1] > M_PI) ? 2.0 * M_PI : 0;
	}
	else if (accMagOrientation[1] < -0.5 * M_PI && gyroOrientation[1] > 0.0) {
		fusedOrientation[1] = (double) (FILTER_COEFFICIENT * gyroOrientation[1] + oneMinusCoeff * (accMagOrientation[1] + 2.0 * M_PI));
		fusedOrientation[1] -= (fusedOrientation[1] > M_PI)? 2.0 * M_PI : 0;
	}
	else {
		fusedOrientation[1] = FILTER_COEFFICIENT * gyroOrientation[1] + oneMinusCoeff * accMagOrientation[1];
	}

	// roll
	if (gyroOrientation[2] < -0.5 * M_PI && accMagOrientation[2] > 0.0) {
		fusedOrientation[2] = (double) (FILTER_COEFFICIENT * (gyroOrientation[2] + 2.0 * M_PI) + oneMinusCoeff * accMagOrientation[2]);
		fusedOrientation[2] -= (fusedOrientation[2] > M_PI) ? 2.0 * M_PI : 0;
	}
	else if (accMagOrientation[2] < -0.5 * M_PI && gyroOrientation[2] > 0.0) {
		fusedOrientation[2] = (double) (FILTER_COEFFICIENT * gyroOrientation[2] + oneMinusCoeff * (accMagOrientation[2] + 2.0 * M_PI));
		fusedOrientation[2] -= (fusedOrientation[2] > M_PI)? 2.0 * M_PI : 0;
	}
	else {
		fusedOrientation[2] = FILTER_COEFFICIENT * gyroOrientation[2] + oneMinusCoeff * accMagOrientation[2];
	}

	// overwrite gyro matrix and orientation with fused orientation
	// to comensate gyro drift
	getRotationMatrixFromOrientation(gyroMatrix, fusedOrientation);
	memcpy(gyroOrientation, fusedOrientation, sizeof(gyroOrientation));
}

// Main loop
void razor_loop(void)
{
	razor_loop_input();

	// Time to read the sensors again?
	if ((timer_systime() - timestamp) >= OUTPUT__DATA_INTERVAL)
	{
		timestamp_old = timestamp;
		timestamp = timer_systime();
		if (timestamp > timestamp_old)
			G_Dt = (double)MS2S(timestamp - timestamp_old); // Real time of loop run. We use this on the DCM algorithm (gyro integration time)
		else
			G_Dt = 0.0;

		// Update sensor readings
		read_sensors();

		switch (output_mode)
		{
			case OUTPUT__MODE_CALIBRATE_SENSORS:
			// We're in calibration mode
			{
				check_reset_calibration_session();  // Check if this session needs a reset
				if (output_stream_on || output_single_on) output_calibration(curr_calibration_sensor);
				break;
			}
			case OUTPUT__MODE_YPR_RAZOR:
			{
				// Apply sensor calibration
				compensate_sensor_errors();

				// Run DCM algorithm
				Compass_Heading(); // Calculate magnetic heading
				Matrix_update();
				Normalize();
				Drift_correction();
				Euler_angles();

				if (output_stream_on || output_single_on) output_angles();
				break;
			}
			case OUTPUT__MODE_YPR_MADGWICK:
			{
				compensate_sensor_errors();
				gyro[0] = GYRO_SCALED_RAD(gyro[0]);
				gyro[1] = GYRO_SCALED_RAD(gyro[1]);
				gyro[2] = GYRO_SCALED_RAD(gyro[2]);
				// acc and mag already scaled in compensate_sensor_errors()

				// Apply Madgwick algorithm
				MadgwickAHRSupdate(gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2], magnetom[0], magnetom[1], magnetom[2]);
				//MadgwickAHRSupdate(gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2], magnetom[0], magnetom[1], magnetom[2]);
				//MadgwickAHRSupdate(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, magnetom[0], magnetom[1], magnetom[2]);
				MadgwickYawPitchRoll(&yaw, &pitch, &roll);
				if (output_stream_on || output_single_on) output_angles();
				break;
			}
			case OUTPUT__MODE_YPR_FREEIMU:
			{
				compensate_sensor_errors();
				gyro[0] = GYRO_SCALED_RAD(gyro[0]);
				gyro[1] = GYRO_SCALED_RAD(gyro[1]);
				gyro[2] = GYRO_SCALED_RAD(gyro[2]);
				// acc and mag already scaled in compensate_sensor_errors()

				// get sensorManager and initialise sensor listeners
				//mSensorManager = (SensorManager) this.getSystemService(SENSOR_SERVICE);
				//initListeners();

				// wait for one second until gyroscope and magnetometer/accelerometer
				// data is initialised then scedule the complementary filter task
				//fuseTimer.scheduleAtFixedRate(new calculateFusedOrientationTask(), 1000, TIME_CONSTANT);

				//public void onSensorChanged(SensorEvent event) {
					//switch(event.sensor.getType()) {
						//case Sensor.TYPE_ACCELEROMETER:
						// copy new accelerometer data into accel array and calculate orientation
						//System.arraycopy(event.values, 0, accel, 0, 3);
						calculateAccMagOrientation();

						//case Sensor.TYPE_GYROSCOPE:
						// process gyro data
						//gyroFunction(event);
						gyroFunction();

						//case Sensor.TYPE_MAGNETIC_FIELD:
						// copy new magnetometer data into magnet array
						//System.arraycopy(event.values, 0, magnet, 0, 3);

						calculateFusedOrientation();

						yaw = -fusedOrientation[0];
						roll = -fusedOrientation[1];
						pitch = fusedOrientation[2];

				//if (output_stream_on || output_single_on) output_angles_freedom();
				if (output_stream_on || output_single_on) output_angles();
				break;
			}
			case OUTPUT__MODE_SENSORS_RAW:
			case OUTPUT__MODE_SENSORS_CALIB:
			case OUTPUT__MODE_SENSORS_BOTH:
			{
				if (output_stream_on || output_single_on) output_sensors();
				break;
			}
		}

		output_single_on = false;

		#if DEBUG__PRINT_LOOP_TIME == true
		//printf("loop time (ms) = %lu\r\n", timer_systime() - timestamp);
		// Not really useful since RTC measures only 4 ms.
		#endif
	}
	#if DEBUG__PRINT_LOOP_TIME == true
	else
	{
		printf("waiting...\r\n");
	}
	#endif

	return;
}
