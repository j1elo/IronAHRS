/*
 * Created: 16/09/2012 17:43:14
 * Author: Juan Navarro Moreno
 */

#ifndef SERIAL_H_
#define SERIAL_H_

#include "external/Serial/WinSerial.h"
#include <string>  // std::string

bool serial_init(const std::string& com_port);
void serial_finish();
bool serial_readline(std::string& out_line);

// Global access to serial port.
extern CSerial com;

#endif // SERIAL_H_
