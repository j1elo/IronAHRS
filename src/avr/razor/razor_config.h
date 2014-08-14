/*
 * razor_config.h
 *
 * Created: 29/06/2012 20:49:22
 * Author: Juan Navarro Moreno
 */

#ifndef RAZOR_CONFIG_H_
#define RAZOR_CONFIG_H_

#include <stdbool.h>

#define REINIT_GLOBALS 1


/*****************************************************************/
/*********** USER SETUP AREA! Set your options here! *************/
/*****************************************************************/

// Sensor data output interval in milliseconds
// This may not work, if faster than 20ms (=50Hz)
// Code is tuned for 20ms, so better leave it like that
#define OUTPUT__DATA_INTERVAL 8  // in milliseconds

// Output mode
//typedef enum {
	//OUTPUT__MODE_CALIBRATE_SENSORS,  // Outputs sensor min/max values as text for manual calibration.
	//OUTPUT__MODE_SENSORS_RAW,  // Raw (uncalibrated) sensor values:
							//// "AccX,AccY,AccZ,GyroX,GyroY,GyroZ,MagX,MagY,MagZ".
	//OUTPUT__MODE_SENSORS_CALIB,  // Calibrated sensor values:
							//// "AccX,AccY,AccZ,GyroX,GyroY,GyroZ,MagX,MagY,MagZ".
	//OUTPUT__MODE_SENSORS_BOTH,  // Raw and calibrated sensor values:
							//// "AccXraw,AccYraw,AccZraw,GyroXraw,GyroYraw,GyroZraw,MagXraw,MagYraw,MagZraw",
							//// "AccXcal,AccYcal,AccZcal,GyroXcal,GyroYcal,GyroZcal,MagXcal,MagYcal,MagZcal".
	//OUTPUT__MODE_YPR_RAZOR,  // Angle values: "#YPR=Yaw,Pitch,Roll".
	//OUTPUT__MODE_YPR_MADGWICK,
	//OUTPUT__MODE_YPR_FREEIMU,
//} output_mode_t;
extern output_mode_t output_mode;
#define STARTUP_OUTPUT_MODE OUTPUT__MODE_YPR_FREEIMU

extern bool output_serialchart;

// Output format
//typedef enum {
	//OUTPUT__FORMAT_BINARY,  // Output values as binary floats.
	//OUTPUT__FORMAT_TEXT
//} output_format_t;
extern output_format_t output_format;
#define STARTUP_OUTPUT_FORMAT OUTPUT__FORMAT_TEXT

// Select if serial continuous streaming output is enabled per default on startup.
#define OUTPUT__STARTUP_STREAM_ON true  // true or false

// If set true, an error message will be output if we fail to read sensor data.
// Message format: "!ERR: reading <sensor>", followed by "\r\n".
extern char output_errors;  // true or false






// Magnetometer (extended calibration)
// Uncommend to use extended magnetometer calibration (compensates hard & soft iron errors)
//#define CALIBRATION__MAGN_USE_EXTENDED true
extern double magn_ellipsoid_center[3];
extern double magn_ellipsoid_transform[3][3];




// DEBUG OPTIONS
/*****************************************************************/

// Print elapsed time after each I/O loop
#define DEBUG__PRINT_LOOP_TIME false


/*****************************************************************/
/****************** END OF USER SETUP AREA!  *********************/
/*****************************************************************/

#endif // RAZOR_CONFIG_H_
