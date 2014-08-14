/*
 * razor.h
 *
 * Created: 29/06/2012 21:08:50
 * Author: Juan Navarro Moreno
 */

#ifndef RAZOR_H_
#define RAZOR_H_

#include "razor_config.h"









#include <stdio.h>




#include <stdbool.h>
#include "../board/board.h"
#include "../board/serial.h"
#include "../board/timer.h"
#include "razor_config.h"
#include "razor_sensors.h"

#include "razor_compass.h"
#include "razor_output.h"
#include "razor_dcm.h"



void razor_loop(void);

// Sensor variables
extern double accel[3];  // Actually stores the NEGATED acceleration (equals gravity, if board not moving).
extern double accel_min[3];
extern double accel_max[3];

extern double magnetom[3];
extern double magnetom_min[3];
extern double magnetom_max[3];
extern double magnetom_tmp[3];

extern double gyro[3];
extern double gyro_average[3];
extern int gyro_num_samples;

// DCM variables
extern double MAG_Heading;
extern double Accel_Vector[3];
extern double Gyro_Vector[3];
extern double Omega_Vector[3];
extern double Omega_P[3];
extern double Omega_I[3];
extern double Omega[3];
extern double errorRollPitch[3];
extern double errorYaw[3];
extern double DCM_Matrix[3][3];
extern double Update_Matrix[3][3];
extern double Temporary_Matrix[3][3];

// Euler angles
extern double yaw;
extern double pitch;
extern double roll;

// DCM timing in the main loop
extern double G_Dt; // Integration time for DCM algorithm

#endif // RAZOR_H_
