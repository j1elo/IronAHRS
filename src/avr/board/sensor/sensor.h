/*
 * Sensors.h
 *
 * Created: 27/06/2012 18:21:54
 * Author: Juan Navarro
 */

#ifndef SENSOR_H_
#define SENSOR_H_

#define DEBUG_SENSOR 0

#include "accelerometer_adxl345.h"
#include "gyroscope_l3g4200d.h"
#include "magnetometer_hmc5883l.h"


// Check for 9 Degree Of Freedom.
void sensor_init(void); // equivalent to the old CheckforMEMSensors()

#endif // SENSOR_H_
