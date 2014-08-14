/*
 * razor.h
 *
 * Created: 12/09/2012 16:56:45
 * Author: Juan Navarro
 */

#ifndef RAZOR_H_
#define RAZOR_H_

#include <stdint.h>

// When set to true, gyro drift correction will not be applied
#define DEBUG__NO_DRIFT_CORRECTION 0

void razor_init(uint8_t dt_ms);
void razor_heading(double out_YawPitchRoll_rad[3], /*const*/ double in_AccGyroMag_calib[3][3]);

#endif // RAZOR_H_
