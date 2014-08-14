/*
 * Created: 12/11/2012 14:12
 * Author: Juan Navarro Moreno
 */

#include "config.h"
#include <iostream>  // std::cout - basic I/O streams.

// Public access to configuration values.
DataSource_t config_DataSource;
std::string config_InputFile;
std::string config_SerialPort;
bool config_AgmRawOutput;
std::string config_AgmRawOutputFile;
bool config_AgmCalibOutput;
std::string config_AgmCalibOutputFile;
bool config_YprOutput;
std::string config_YprOutputFile;
bool config_SocketOutput;
long config_SocketOutputPort;

// Global access to config handler.
CSimpleIniCaseA config;


void config_init(const std::string& path)
{
	config.SetSpaces(true);

	if (config.LoadFile(path.c_str()) == SI_OK) {
		long value = config.GetLongValue("General", "DataSource");
		config_DataSource = (value ? INPUT_FILE : INPUT_SERIAL);

		config_InputFile = config.GetValue("General", "InputFile");
		config_SerialPort = config.GetValue("General", "SerialPort");
		config_AgmRawOutput = config.GetBoolValue("General", "AgmRawOutput");
		config_AgmRawOutputFile = config.GetValue("General", "AgmRawOutputFile");
		config_AgmCalibOutput = config.GetBoolValue("General", "AgmCalibOutput");
		config_AgmCalibOutputFile = config.GetValue("General", "AgmCalibOutputFile");
		config_YprOutput = config.GetBoolValue("General", "YprOutput");
		config_YprOutputFile = config.GetValue("General", "YprOutputFile");
		config_SocketOutput = config.GetBoolValue("General", "SocketOutput");
		config_SocketOutputPort = config.GetLongValue("General", "SocketOutputPort");

		std::cout << "Configuration file '" << path << "' loaded." << std::endl;
	}
	else {
		// Generate a default configuration file.
		config.SetLongValue("General", "DataSource", 1, "; 0: Serial port. 1: Text file.");
		config.SetValue("General", "InputFile", "..\\etc\\data\\video_AGM_RAW.txt");
		config.SetValue("General", "SerialPort", "COM6");
		config.SetValue("General", "AgmRawOutputFile", "auto", "; 'auto' for automatic naming.");
		config.SetValue("General", "AgmCalibOutputFile", "auto");
		config.SetValue("General", "YprOutputFile", "auto");
		config.SetBoolValue("General", "AgmRawOutput", true);
		config.SetBoolValue("General", "AgmCalibOutput", false);
		config.SetBoolValue("General", "YprOutput", true);
		config.SetBoolValue("General", "SocketOutput", false);
		config.SetLongValue("General", "SocketOutputPort", 2042);
		config.SaveFile(path.c_str());
		config.Reset();

		std::cout << "Configuration file '" << path << "' NOT FOUND. Default created." << std::endl;
		return config_init(path); // Call itself again; now it will find the config file.
	}

	/*std::cout << "DataSource: " << config_DataSource << std::endl;
	std::cout << "InputFile: " << config_InputFile << std::endl;
	std::cout << "SerialPort: " << config_SerialPort << std::endl;*/
}
