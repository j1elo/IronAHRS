/*
 * razor_output.c
 *
 * Created: 29/06/2012 21:04:43
 *  Author: Juan
 */

#include <math.h>
#include "razor.h"


// Output angles: yaw, pitch, roll
void output_angles()
{
	if (output_format == OUTPUT__FORMAT_BINARY)
	{
		double ypr[3];
		ypr[0] = RAD2DEG(yaw);
		ypr[1] = RAD2DEG(pitch);
		ypr[2] = RAD2DEG(roll);
		serial_write((uint8_t*)ypr, 12);  // No new-line
	}
	else if (output_format == OUTPUT_FORMAT_TEXT)
	{
		if (output_serialchart)
			printf("%+.8f,%+.8f,%+.8f\r\n", RAD2DEG(yaw), RAD2DEG(pitch), RAD2DEG(roll));
		else
			printf("#YPR=%+.8f,%+.8f,%+.8f\r\n", RAD2DEG(yaw), RAD2DEG(pitch), RAD2DEG(roll));
	}
}

void output_angles_freedom()
{
	if (output_format == OUTPUT__FORMAT_BINARY)
	{
		double ypr[3];
		ypr[0] = yaw;
		ypr[1] = pitch;
		ypr[2] = roll;
		serial_write((uint8_t*)ypr, 12);  // No new-line
	}
	else if (output_format == OUTPUT_FORMAT_TEXT)
	{
		if (output_serialchart)
			printf("%+.8f,%+.8f,%+.8f\r\n", yaw, pitch, roll);
		else
			printf("#YPR=%+.8f,%+.8f,%+.8f\r\n", yaw, pitch, roll);
	}
}

void output_calibration(int calibration_sensor)
{
	if (calibration_sensor == 0)  // Accelerometer
	{
		// Output MIN/MAX values
		printf("accel x,y,z (min/max) = ");
		for (int i = 0; i < 3; i++) {
			if (accel[i] < accel_min[i]) accel_min[i] = accel[i];
			if (accel[i] > accel_max[i]) accel_max[i] = accel[i];
			printf("%+.8f/%+.8f", accel_min[i], accel_max[i]);
			if (i < 2) printf("  ");
			else printf("\r\n");
		}
	}
	else if (calibration_sensor == 1)  // Magnetometer
	{
		// Output MIN/MAX values
		printf("magn x,y,z (min/max) = ");
		for (int i = 0; i < 3; i++) {
			if (magnetom[i] < magnetom_min[i]) magnetom_min[i] = magnetom[i];
			if (magnetom[i] > magnetom_max[i]) magnetom_max[i] = magnetom[i];
			printf("%+.8f/%+.8f", magnetom_min[i], magnetom_max[i]);
			if (i < 2) printf("  ");
			else printf("\r\n");
		}
	}
	else if (calibration_sensor == 2)  // Gyroscope
	{
		// Average gyro values
		for (int i = 0; i < 3; i++)
			gyro_average[i] += gyro[i];
		gyro_num_samples++;

		// Output current and averaged gyroscope values
		printf("gyro x,y,z (current/average) = ");
		for (int i = 0; i < 3; i++) {
			printf("%+.8f/%+.8f", gyro[i], (gyro_average[i] / (double)gyro_num_samples));
			if (i < 2) printf("  ");
			else printf("\r\n");
		}
	}
}





void output_sensors()
{
	if (output_mode == OUTPUT__MODE_SENSORS_RAW)
	{
		if (output_format == OUTPUT__FORMAT_BINARY)
			output_sensors_binary();
		else if (output_format == OUTPUT_FORMAT_TEXT)
			output_sensors_text('R');
	}
	else if (output_mode == OUTPUT__MODE_SENSORS_CALIB)
	{
		// Apply sensor calibration
		compensate_sensor_errors();

		if (output_format == OUTPUT__FORMAT_BINARY)
			output_sensors_binary();
		else if (output_format == OUTPUT_FORMAT_TEXT)
			output_sensors_text('C');
	}
	else if (output_mode == OUTPUT__MODE_SENSORS_BOTH)
	{
		if (output_format == OUTPUT__FORMAT_BINARY)
		{
			output_sensors_binary();
			compensate_sensor_errors();
			output_sensors_binary();
		}
		else if (output_format == OUTPUT_FORMAT_TEXT)
		{
			output_sensors_text('R');
			compensate_sensor_errors();
			output_sensors_text('C');
		}
	}
}
