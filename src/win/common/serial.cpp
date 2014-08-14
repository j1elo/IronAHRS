/*
 * Created: 16/09/2012 17:43:14
 * Author: Juan Navarro Moreno
 */

#include "serial.h"
#include <iostream>  // std::cout - basic I/O streams.
#include <cstdint>  // C standard fixed-size integer definitions.
#include <windows.h>  // Win32 API.
#include <tchar.h>  // Win32 API character definitions.

#define BUFFER_SIZE 2048

// Global access to serial port.
CSerial com;


bool serial_init(const std::string& com_port)
{
	LONG lLastError = ERROR_SUCCESS;
	LPCTSTR lpszDevice = (LPCTSTR)com_port.c_str();
	
	// Attempt to open the serial port.
	lLastError = com.Open(lpszDevice, 0, 0, false);
	//lLastError = com.Open(TEXT("COM3"), 0, 0, false);
	if (lLastError != ERROR_SUCCESS) {
		std::cout << "Oops! The device '" << lpszDevice << "' cannot be opened" << std::endl << "(error code: " << lLastError << ")" << std::endl;
		return false;
	}

	// Setup the serial port basics.
	lLastError = com.Setup(
		CSerial::EBaud115200,
		CSerial::EData8,
		CSerial::EParNone,
		CSerial::EStop1);
	if (lLastError != ERROR_SUCCESS) {
		std::cout << "Oops! The device '" << lpszDevice << "' cannot be configured 1" << std::endl << "(error code: " << lLastError << ")" << std::endl;
		return false;
	}

	// Setup XON/XOFF handshaking.
	lLastError = com.SetupHandshaking(CSerial::EHandshakeSoftware);
	if (lLastError != ERROR_SUCCESS) {
		std::cout << "Oops! The device '" << lpszDevice << "' cannot be configured 2" << std::endl << "(error code: " << lLastError << ")" << std::endl;
		return false;
	}

	// Setup blocking Read operations.
	lLastError = com.SetupReadTimeouts(CSerial::EReadTimeoutBlocking);
	if (lLastError != ERROR_SUCCESS) {
		std::cout << "Oops! The device '" << lpszDevice << "' cannot be configured 3" << std::endl << "(error code: " << lLastError << ")" << std::endl;
		return false;
	}

	return true;
}

void serial_finish()
{
	com.Purge();  // Clear input buffer up to here.
	com.Close();
}

bool serial_readline(std::string& out_line)
{
	LONG lLastError = ERROR_SUCCESS;
	bool ok = true;
	uint8_t buffer[BUFFER_SIZE];
	unsigned int i = 0;

	while (true)
	{
		lLastError = com.Read(&buffer[i], 1);
		if (lLastError != ERROR_SUCCESS) {
			// Serial device was disabled or disconnected.
			ok = false;
			break;
		}
		
		if (buffer[i] == '\n') {
			// EOL: mark end of string.
			buffer[i + 1] = '\0';
			ok = true;
			break;
		}
		
		// Check array boundaries.
		if (i + 2 >= BUFFER_SIZE) {
			// Receiving bad data without EOL.
			std::cout << "ERROR: read_line(): serial data never gets EOL!" << std::endl;
			ok = false;
			break;
		}
		i += 1;
	}
	out_line = (const char *)buffer;
	return ok;
}
