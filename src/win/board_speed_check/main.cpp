/*
 * Created: 26/02/2013 19:30:12
 * Author: Juan Navarro Moreno
 */

#include "common/external/GetTimeMs.h"
#include "common/config.h"
#include "common/serial.h"

#include <iostream>  // Basic I/O streams: std::cout
#include <string>  // std::string, std::to_string

#include <cstdlib>  // llabs()
#include <cstdio>  // printf()
#include <cstdint>  // uint64_t

#include <windows.h>  // Win32 API.
#include <tchar.h>  // Win32 API character definitions.

#define BOARD_SETUP_COMMAND "qt5"  // Setup "Test-ClockGuess" mode.

BOOL signal_handler(DWORD fdwCtrlType);
int finish();

// Signal callback function.
// Handle CTRL+C signals.
//BOOL signal_handler(DWORD fdwCtrlType)
BOOL signal_handler(DWORD)
{
	finish();
	std::cout << "CTRL+C received. Exit now!" << std::endl;
	exit(0);
}

bool input_init(DataSource_t source)
{
	if (!serial_init(config_SerialPort)) {
		return false;
	}
	std::cout << "Setup Sensor Board... ";
	com.Write(BOARD_SETUP_COMMAND);
	com.Purge();  // Clear input buffer up to here.
	std::cout << "OK." << std::endl;
	return true;
}

int finish()
{
	serial_finish();
	return 0;
}

int main(int argc, char** argv)
{
	// Set CTRL+C signal handler.
	SetConsoleCtrlHandler((PHANDLER_ROUTINE)signal_handler, TRUE);

    config_init("board_speed_check.cfg");

	if (!input_init(INPUT_SERIAL))
		return finish();

    uint64_t delay = 0;
    uint32_t guess = 1000000;  // Start with 1 MHz guess.
    uint32_t top = 0;
    uint32_t bottom = 0;

    /*
     * Searching algorithm
     *
     * We want to find the uC clock speed in Hz, or cycles per second. So we try to guess the amount
     * of cycles which cause a delay of 1000 milliseconds in the uC.
     * 
     * Use a State Machine which separates between 2 phases:
     * 1) Look for either upper or bottom bounds (states '1' or '2').
     * Starting at 1 MHz guess, we don't know if the real value will be higher or lower, and by how much.
     * 2) Bubble Search (state '3').
     * Once bounds have been found, perform a simple bubble search to find the actual value.
     *
     * Results won't be totally accurate because of the cycles spent by the uC in data I/O and USART handling.
     * Typically the actual clock speed is a bit higher than what found by this procedure.
     * Eg. found value: 10968750 Hz. Actual value: 11059200 Hz. Difference: around 0.8 %.
     */
    unsigned int mode = 0;  // 1: Find Upper Bound; 2: Find Bottom Bound; 3: Bubble Search.

    std::cout << "Guess Hz count for 1000 ms. delay..." << std::endl;
        
    bool working = true;
    while (working)
    {
        std::string guess_str = std::to_string((unsigned long long)guess) + '\n';

        std::string input_line;
        bool ok_readline = true;

        uint64_t t0 = GetTimeMs64();
        com.Write(guess_str.data());
        ok_readline = serial_readline(input_line);
        delay = GetTimeMs64() - t0;

        //std::cout << "Answer from board: " << input_line << std::endl;
        std::cout << "Guess: " << guess << " Hz -> response delay: " << delay << " ms." << std::endl;
                
        if (input_line != "CHECK\r\n") {
            continue;
        }

        if (llabs(signed(1000 - delay)) <= 1)
        {
            // Finished.
            std::cout << "Found value: uC clock is around " << guess << " Hz" << std::endl;
            working = false;
        }
        else if (signed(1000 - delay) > 0)
        {
            std::cout << "We need bigger guess" << std::endl;

            switch (mode) {
            case 0:
            case 1:
                mode = 1;
                bottom = guess;
                guess *= 2;
                top = guess;
                break;
            case 2:
            case 3:
                // We were in Smaller mode! bounds found.
                mode = 3;
                bottom = guess;
                guess = (bottom + top) / 2;
                break;
            }

            //std::cout << "New guess: " << guess << std::endl;
        }
        else if (signed(1000 - delay) < 0)
        {
            std::cout << "We need smaller guess" << std::endl;

            switch (mode) {
            case 0:
            case 2:
                mode = 2;
                top = guess;
                guess /= 2;
                bottom = guess;
                break;
            case 1:
            case 3:
                // We were in Bigger mode! bounds found.
                mode = 3;
                top = guess;
                guess = (bottom + top) / 2;
                break;
            }

            //std::cout << "New guess: " << guess << std::endl;
        }
    }

    std::cout << std::endl << "C'est fini!" << std::endl;
	return finish();
}
