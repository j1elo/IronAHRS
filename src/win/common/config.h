/*
 * Created: 12/11/2012 14:12
 * Author: Juan Navarro Moreno
 */

#ifndef CONFIG_H_
#define CONFIG_H_

#include "external/SimpleINI/SimpleIni.h"
#include <string>  // std::string

typedef enum {
	INPUT_SERIAL = 0,
	INPUT_FILE = 1
} DataSource_t;

void config_init(const std::string& file);

// Public access to configuration values.
extern DataSource_t config_DataSource;
extern std::string config_InputFile;
extern std::string config_SerialPort;
extern bool config_AgmRawOutput;
extern std::string config_AgmRawOutputFile;
extern bool config_AgmCalibOutput;
extern std::string config_AgmCalibOutputFile;
extern bool config_YprOutput;
extern std::string config_YprOutputFile;
extern bool config_SocketOutput;
extern long config_SocketOutputPort;

#endif // SERIAL_H_
