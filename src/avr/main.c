/*
 * main.c
 *
 * Created: 27/06/2012 17:55:10
 * Author: Juan Navarro
 */

/*
NOTE - About timer delays and project configuration.
The module "timer" uses some "Busy Wait" software delays, which expect
proper project configuration at compilation time:
a) F_CPU must be defined as the CPU clock frequency in Hz.
   For example: -DF_CPU=16000000UL
b) Compiler optimization must be enabled.
   At least level 1: -O1.
*/

/*
NOTE - About printf() / scanf() and stdout / stdin.
There are 3 implementations of the base vfprintf() and vfscanf() functions,
differing in binary size:
- Default version implements all the functionality except floating point conversions.
- Minimized version that only implements the very basic integer and string conversions.
  Usage: link with these flags:
  vfprintf(): -Wl,-u,vfprintf -lprintf_min
  vfscanf(): -Wl,-u,vfscanf -lscanf_min -lm
- Full functionality including the floating point conversions.
  Usage: link with these flags:
  vfprintf(): -Wl,-u,vfprintf -lprintf_flt -lm
  vfscanf():  -Wl,-u,vfscanf -lscanf_flt -lm

SEE avr-libc Library Reference - <stdio.h>: Standard IO facilities
    http://www.nongnu.org/avr-libc/user-manual/group__avr__stdio.html
*/

/*
NOTE - About using "volatile" variables in embedded programming.
"volatile" are needed to avoid compiler optimizations in areas where
some other thread or interrupt might modify the data in use.
Read about it carefully.

SEE "Nine ways to break your systems code using volatile"
    http://blog.regehr.org/archives/28

SEE avr-libc Library Reference - "Problems with reordering code"
    http://www.nongnu.org/avr-libc/user-manual/optimization.html#optim_code_reorder
*/

/*
NOTE - About floating-point precision.
The only supported floating point format is 32 bits wide,
for both 'float' and 'double' types.
Maximum precision of 8 fractional digits.

SEE avr-libc Library Reference - FAQ - What registers are used by the C compiler?
    http://www.nongnu.org/avr-libc/user-manual/FAQ.html#faq_reg_usage
*/


// DEBUG variables:
// serial.h -> DEBUG_SERIAL
// timer.h  -> DEBUG_TIMER
// sensor.h -> DEBUG_SENSOR

#include "board/board.h"
#include "board/serial.h"
#include "board/timer.h"
#include "board/sensor/sensor.h"
#include "heading/common.h"
#include "heading/razor.h"
#include "test/tests.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h> // FILE, stream facilities.

typedef enum {
	MODE_SENSORS_RAW,
	MODE_SENSORS_CALIB,
	MODE_SENSORS_BOTH,
	MODE_YPR
} output_mode_t;

typedef enum {
	FORMAT_TEXT,
	FORMAT_BINARY
} output_format_t;

typedef enum {
	FILTER_RAZOR,
	FILTER_MADGWICK,
	FILTER_FREEIMU
} heading_filter_t;

output_mode_t g_output_mode;
output_format_t g_output_format;
heading_filter_t g_heading_filter;

// Sensor acquisition frequency interval in milliseconds.
uint8_t g_loop_period;


// Functions for enabling stdin/stdout operations through serial port.
static int serial_getchar(FILE* stream)
{ return (int)serial_read(); }
static int serial_putchar(char c, FILE* stream)
{ serial_write((uint8_t*)&c, 1); return 0; }
static FILE mystdin = FDEV_SETUP_STREAM(NULL, serial_getchar, _FDEV_SETUP_READ);
static FILE mystdout = FDEV_SETUP_STREAM(serial_putchar, NULL, _FDEV_SETUP_WRITE);


// Blink every specified ms.
void green_led_blink(uint16_t ms)
{
	if (*(volatile uint16_t *)&timer_mscountdown_1 == 0) {
		timer_mscountdown_1 = ms;
		static bool green_led_enable = false;
		if (green_led_enable) {
			board_green_led(false);
			green_led_enable = false;
		}
		else {
			board_green_led(true);
			green_led_enable = true;
		}
	}
}

// Process incoming control queries.
void get_input_command(void)
{
	if (serial_available() < 1)  // 1, so we can pause with just one key press.
		return;

	if (serial_read() != 'q')
		return;

	char command = serial_read();
	switch (command)
	{
		case 'o':
		// Output mode.
		{
			char output_param = serial_read();
			switch (output_param)
			{
				case 'c':
				// Go to Calibration mode.
				{
					//g_output_mode = OUTPUT__MODE_CALIBRATE_SENSORS;
					//reset_calibration_session_flag = true;
					break;
				}
				case 'n':
				// Calibrate Next sensor.
				{
					//curr_calibration_sensor = (curr_calibration_sensor + 1) % 3;
					//reset_calibration_session_flag = true;
					break;
				}
				case 's':
				// Output sensor values.
				{
					char values_param = serial_read();
					char format_param = serial_read();
					switch (values_param)
					{
						case 'r':
						// Output Raw sensor values.
						{
							g_output_mode = MODE_SENSORS_RAW;
							break;
						}
						case 'c':
						// Output Calibrated sensor values.
						{
							g_output_mode = MODE_SENSORS_CALIB;
							break;
						}
						case 'b':
						// Output Both sensor values (raw and calibrated).
						{
							g_output_mode = MODE_SENSORS_BOTH;
							break;
						}
					}
					switch (format_param)
					{
						case 't':
						// Output values as Text.
						{
							g_output_format = FORMAT_TEXT;
							break;
						}
						case 'b':
						// Output values in Binary format.
						{
							g_output_format = FORMAT_BINARY;
							break;
						}
					}
					break;
				}
				case 'b':
				// Output angles in Binary format.
				{
					g_output_mode = MODE_YPR;
					g_output_format = FORMAT_BINARY;

					// Do not change the heading filter.
					// (Change manually through serial Terminal).
					// g_heading_filter = FILTER_RAZOR;
					break;
				}
				case 't':
				// Output angles in Text format.
				{
					g_output_mode = MODE_YPR;
					g_output_format = FORMAT_TEXT;
					g_heading_filter = FILTER_RAZOR;
					razor_init(g_loop_period);
					break;
				}
				case 'm':
				{
					g_output_mode = MODE_YPR;
					g_output_format = FORMAT_TEXT;
					g_heading_filter = FILTER_MADGWICK;
					break;
				}
				case 'f':
				{
					g_output_mode = MODE_YPR;
					g_output_format = FORMAT_TEXT;
					g_heading_filter = FILTER_FREEIMU;
					break;
				}
				case '0':
				{
					// Disable continuous streaming output
					//turn_output_stream_off();
					//reset_calibration_session_flag = true;
					break;
				}
				case '1':
				{
					// Enable continuous streaming output
					//reset_calibration_session_flag = true;
					//turn_output_stream_on();
					break;
				}
				case 'e':
				// _e_rror output settings
				{
					//char error_param = readChar();
					//if (error_param == '0') output_errors = false;
					//else if (error_param == '1') output_errors = true;
					//else if (error_param == 'c') // get error count
						//printf("#AMG-ERR: %d, %d, %d\r\n", num_accel_errors, num_magn_errors, num_gyro_errors);
					break;
				}
			}
			break;
		}
		case 'f':
		// request one output _f_rame
		{
			//output_single_on = true;
			break;
		}
		case 's':
		// Synch request.
		{
			char id0 = serial_read();  // Read ID.
			char id1 = serial_read();
			printf("#SYNCH%c%c\r\n", id0, id1);  // Reply with synch message.
			break;
		}
		case 'b':
		// Launch the embedded STK500v2 bootloader.
		{
			stk500v2_bootloader();
			break;
		}
		case 'r':
		// Repeat sensor initialization.
		{
			sensor_init();
			break;
		}
        case 't':
        // Test mode.
        {
            char test_param = serial_read();
            switch (test_param)
            {
                case '1':
                {
                    serial_buffer_test();
                    break;
                }
                case '2':
                {
                    serial_test();
                    break;
                }
                case '5':
                {
                    // Guess uC clock speed.
                    clock_speed_guess();
                    break;
                }
            }
            break;
        }
	}
}

void get_sensor_data(double out_AccGyroMag_raw[3][3])
{
	//TODO ahora mismo esto es algoritmo-dependant.

	int16_t axis[3];

	acc_read_axis(axis);
	// Map X,Y,Z axis between physical board and logical algorithm.
	// No multiply by -1 for coordinate system transformation here, because of double negation:
	// we want the gravity vector, which is negated acceleration vector.
	out_AccGyroMag_raw[0][0] = 1.0 * axis[1];  // X axis (internal sensor -Y axis).
	out_AccGyroMag_raw[0][1] = 1.0 * axis[0];  // Y axis (internal sensor -X axis).
	out_AccGyroMag_raw[0][2] = 1.0 * axis[2];  // Z axis (internal sensor -Z axis).

	gyro_read_axis(axis);
	// Map X,Y,Z axis between physical board and logical algorithm.
	out_AccGyroMag_raw[1][0] = -1.0 * axis[0];  // X axis (internal sensor -X axis).
	out_AccGyroMag_raw[1][1] = -1.0 * axis[1];  // Y axis (internal sensor -Y axis).
	out_AccGyroMag_raw[1][2] =  1.0 * axis[2];  // Z axis (internal sensor  Z axis).

	mag_read_axis(axis);
	// Map X,Y,Z axis between physical board and logical algorithm.
	out_AccGyroMag_raw[2][0] =  1.0 * axis[1];  // X axis (internal sensor  Y axis).
	out_AccGyroMag_raw[2][1] =  1.0 * axis[0];  // Y axis (internal sensor  X axis).
	out_AccGyroMag_raw[2][2] = -1.0 * axis[2];  // Z axis (internal sensor -Z axis).
}

void radians_to_degrees(double out_YawPitchRoll_deg[3], /*const*/ double in_YawPitchRoll_rad[3])
{
	for (uint8_t i = 0; i < 3; i++) {
		out_YawPitchRoll_deg[i] = rad_to_deg(in_YawPitchRoll_rad[i]);
	}
}

void output_sensors_text(/*const*/ double in_AccGyroMag[3][3])
{
	printf("%+.8f,%+.8f,%+.8f,%+.8f,%+.8f,%+.8f,%+.8f,%+.8f,%+.8f\r\n",
		in_AccGyroMag[0][0],
		in_AccGyroMag[0][1],
		in_AccGyroMag[0][2],
		in_AccGyroMag[1][0],
		in_AccGyroMag[1][1],
		in_AccGyroMag[1][2],
		in_AccGyroMag[2][0],
		in_AccGyroMag[2][1],
		in_AccGyroMag[2][2]);
}

void output_sensors_binary(/*const*/ double in_AccGyroMag[3][3])
{
	serial_write((uint8_t *)in_AccGyroMag, 3 * 3 * sizeof(double));
}

void output_ypr_text(/*const*/ double in_YawPitchRoll_deg[3])
{
	printf("%+.8f,%+.8f,%+.8f\r\n",
		in_YawPitchRoll_deg[0],
		in_YawPitchRoll_deg[1],
		in_YawPitchRoll_deg[2]);
}

void output_ypr_binary(/*const*/ double in_YawPitchRoll_deg[3])
{
	serial_write((uint8_t *)in_YawPitchRoll_deg, 3 * sizeof(double));
}

int main(void)
{
	// Enable stdin/stdout operations through serial port.
	stdin = &mystdin;
	stdout = &mystdout;

	// Initialize IMU6410 sensor board.
	board_init();
	board_red_led(true);
	printf("\r\n> ");

	// Sensor acquisition frequency interval in milliseconds.
	g_loop_period = 20;  // 20 ms = 50 Hz.

	// Initialize application state.
	// Same as user command "qot".
	// TODO: move get_input_command() and actions to main_menu.c
	g_output_mode = MODE_YPR;
	g_output_format = FORMAT_TEXT;
	g_heading_filter = FILTER_RAZOR;
	razor_init(g_loop_period);

	uint32_t timestamp;
	uint32_t timestamp_old = 0;

	while (true)
	{
		get_input_command();

		timestamp = timer_systime();
		if (timestamp - timestamp_old >= g_loop_period)
		{
			timestamp_old = timestamp;

			double agm_raw[3][3];  // [Acc, Gyro, Mag] uncalibrated (biased) data.
			get_sensor_data(agm_raw);

			if (g_output_mode == MODE_SENSORS_RAW || g_output_mode == MODE_SENSORS_BOTH) {
				if (g_output_format == FORMAT_TEXT)
					output_sensors_text(agm_raw);
				else if (g_output_format == FORMAT_BINARY)
					output_sensors_binary(agm_raw);

				if (g_output_mode != MODE_SENSORS_BOTH)
					continue;
			}

			double agm_calib[3][3];  // [Acc, Gyro, Mag] calibrated data.
			sensor_unbias_scale(agm_calib, agm_raw);

			if (g_output_mode == MODE_SENSORS_CALIB || g_output_mode == MODE_SENSORS_BOTH) {
				if (g_output_format == FORMAT_TEXT)
					output_sensors_text(agm_calib);
				else if (g_output_format == FORMAT_BINARY)
					output_sensors_binary(agm_calib);

				continue;
			}

			double ypr_rad[3];  // [Yaw, Pitch, Roll] angles in radians.
			if (g_heading_filter == FILTER_RAZOR)
				razor_heading(ypr_rad, agm_calib);
			else if (g_heading_filter == FILTER_MADGWICK)
				//madgwick_heading(ypr_rad, agm_raw);
				razor_heading(ypr_rad, agm_calib);
			else if (g_heading_filter == FILTER_FREEIMU)
				//freeimu_heading(ypr_rad, agm_raw);
				razor_heading(ypr_rad, agm_calib);

			double ypr_deg[3];  // [Yaw, Pitch, Roll] angles in degrees.
			radians_to_degrees(ypr_deg, ypr_rad);

			if (g_output_format == FORMAT_TEXT)
				output_ypr_text(ypr_deg);
			else if (g_output_format == FORMAT_BINARY)
				output_ypr_binary(ypr_deg);
		}

		// Blink Green LED slowly to show AVR is alive.
		green_led_blink(500);
	}
	return 0;
}
