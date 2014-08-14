/*
 * Created: 16/09/2012 17:43:14
 * Author: Juan Navarro Moreno
 */

#include "common/external/Socket/WinSocket.h" // This MUST be the first #include
#include "common/external/GetTimeMs.h"

#include "common/config.h"
#include "common/serial.h"
#include "heading/common.h"
#include "heading/razor.h"

#include <iostream>  // Basic I/O streams: std::cout
#include <sstream>  // I/O string streams: std::ostringstream
#include <fstream>  // I/O file streams (std::ifstream, std::ofstream).
#include <string>  // std::string
#include <vector>  // std::vector
#include <regex>  // std::regex_search()

#include <cassert>
#include <cstdio>  // printf()
#include <cmath>  // Numerics library (ceil).
#include <ctime>  // Time Library.

#include <windows.h>  // Win32 API.
#include <tchar.h>  // Win32 API character definitions.

#define BOARD_SETUP_COMMAND "qosrt"  // Setup "Sensor-Raw-Text" output mode.

BOOL signal_handler(DWORD fdwCtrlType);
LRESULT CALLBACK WindowProc(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam);
int finish();

// Input/output file streams.
std::ifstream agmraw_file_in;
std::ofstream agmraw_file_out;
std::ofstream agmcalib_file_out;
std::ofstream ypr_file_out;

// Output socket stream.
Socket* skt_conn = NULL;
SocketServer* skt_server = NULL;


// Signal callback function.
// Handle CTRL+C signals.
//BOOL signal_handler(DWORD fdwCtrlType)
BOOL signal_handler(DWORD)
{
	finish();
	std::cout << "CTRL+C received. Exit now!" << std::endl;
	exit(0);
}

// WinAPI WindowProc callback function.
// Handle WM_SETTINGCHANGE messages.
LRESULT CALLBACK WindowProc(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam)
{
	// See http://msdn.microsoft.com/en-us/library/windows/desktop/ff381408(v=vs.85).aspx
	// and http://msdn.microsoft.com/en-us/library/windows/desktop/ms725497(v=vs.85).aspx
	switch (uMsg) {
		case WM_SETTINGCHANGE:
			UINT uiAction = (UINT)wParam;
			char* area = (char*)lParam;
			printf("CALLBACK: WindowProc(): uiAction %u, area '%s'\n", uiAction, area);
			break;
	}
	return DefWindowProc(hWnd, uMsg, wParam, lParam);
}

bool input_init(DataSource_t source)
{
	if (source == INPUT_SERIAL) {
		if (!serial_init(config_SerialPort)) {
			return false;
		}
		std::cout << "Setup Sensor Board... ";
		com.Write(BOARD_SETUP_COMMAND);
		com.Purge();  // Clear input buffer up to here.
		std::cout << "OK." << std::endl;
	}
	else if (source == INPUT_FILE) {
		// Output file streams.
		agmraw_file_in.open(config_InputFile);
		if (!agmraw_file_in.is_open()) {
			std::cout << "Error opening file '" << config_InputFile << "'." << std::endl;
			return false;
		}
	}
	return true;
}

bool input_readline(std::string& out_line)
{
	bool ok = true;

	if (config_DataSource == INPUT_SERIAL) {
		ok = serial_readline(out_line);
	}
	else if (config_DataSource == INPUT_FILE) {
		if (agmraw_file_in.good()) {
			std::getline(agmraw_file_in, out_line);
			out_line.append("\r\n");  // ADHOC recover characters discarded by getline().
		}
		else {
			std::cout << "End Of File reached on input file." << std::endl;
			ok = false;
		}
	}
	return ok;
}

int finish()
{
	serial_finish();
	if (agmraw_file_in.is_open())
		agmraw_file_in.close();
	if (agmraw_file_out.is_open())
		agmraw_file_out.close();
	if (agmcalib_file_out.is_open())
		agmcalib_file_out.close();
	if (ypr_file_out.is_open())
		ypr_file_out.close();

	if (skt_conn) skt_conn->Close();
	if (skt_server) skt_server->Close();

	return 0;
}

// Get floating-point numbers from an input text line.
// Parses strings like "+11.11,-22.22,-33.33\r\n".
bool parse_line(std::vector<double>& out_data, const std::string& in_line)
{
	bool ok = false;

	// Prepare the regular expression used for obtaining data.
	// See http://www.regular-expressions.info/floatingpoint.html
	//std::regex rex_pattern("([-+]\\d+\\.\\d+)(?:,|\\r\\n$)");
	std::regex rex_pattern("([-+]?\\d*\\.?\\d+)(?:,|\\r\\n$)");
	std::smatch rex_match;

	std::sregex_iterator rex_it(in_line.begin(), in_line.end(), rex_pattern);
	std::sregex_iterator rex_end;

	while (rex_it != rex_end) {
		std::istringstream iss((*rex_it)[1]);  // [1] contains the useful match.
		double x;
		if (!(iss >> x)) {
			// This shouldn't happen. There is NO proper error handling here.
			std::cout << "FATAL: parse_line(): converting input." << std::endl;
			x = 0.0;
		}
		out_data.push_back(x);

		// Signal "ok" when all the input string has been consumed.
		if (rex_it->prefix().str().empty() && rex_it->suffix().str().empty())
			ok = true;

		++rex_it;
	}

	if (!ok) {
		std::cout << "ERROR: parse_line(): wrong input format." << std::endl;
		//out_data.clear();
	}
	return ok;
}

void radians_to_degrees(double out_YawPitchRoll_deg[3], /*const*/ double in_YawPitchRoll_rad[3])
{
	for (uint8_t i = 0; i < 3; i++) {
		out_YawPitchRoll_deg[i] = rad_to_deg(in_YawPitchRoll_rad[i]);
	}
}

// Print a string with the data.
void vector_to_str(std::string& out_str, const std::vector<double>& data, const std::string& header="")
{
	std::ostringstream oss;
	oss << header;

	auto iter = data.begin();
	for (;;) {
		oss << *iter;
		if (++iter == data.end()) break;
		oss << ",";
	}
	out_str = oss.str();
}

// Print a string with the data.
template<unsigned int ROWS, unsigned int COLS>
void array2d_to_str(std::string& out_str, double (&data)[ROWS][COLS], const std::string& header="")
{
	std::ostringstream oss;
	oss << header;

	for (unsigned i = 0; i < ROWS; i++) {
		for (unsigned j = 0; j < COLS; j++) {
			oss << data[i][j];
			if (i+1 == ROWS && j+1 == COLS) break;
			oss << ",";
		}
	}
	out_str = oss.str();
}

void vector_to_array(double out[3][3], const std::vector<double>& in)
{
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			out[i][j] = in[3 * i + j];
		}
	}
}

int main(int argc, char** argv)
{
	// Set CTRL+C signal handler.
	SetConsoleCtrlHandler((PHANDLER_ROUTINE)signal_handler, TRUE);

	config_init("heading_bench.cfg");

	if (!input_init(config_DataSource))
		return finish();

	// Get local time.
	time_t rawtime;
	time(&rawtime);
	struct tm timeinfo;
	localtime_s(&timeinfo, &rawtime);
	char timestr[20];
	strftime(timestr, 20, "%Y-%m-%d_%H-%M-%S", &timeinfo);

	// Output file streams.
	if (config_AgmRawOutput) {
		std::string file_name;
		if (config_AgmRawOutputFile == "auto") {
			file_name = std::string(timestr) + std::string("_AGM_RAW.txt");
		}
		else {
			file_name = config_AgmRawOutputFile;
		}
		agmraw_file_out.open(file_name, std::ios::trunc);
		if (!agmraw_file_out.is_open()) {
			std::cout << "Error opening file '" << file_name << "'." << std::endl;
			return finish();
		}
	}
	if (config_AgmCalibOutput) {
		std::string file_name;
		if (config_AgmCalibOutputFile == "auto") {
			file_name = std::string(timestr) + std::string("_AGM_CALIB.txt");
		}
		else {
			file_name = config_AgmCalibOutputFile;
		}
		agmcalib_file_out.open(file_name, std::ios::trunc);
		if (!agmcalib_file_out.is_open()) {
			std::cout << "Error opening file '" << file_name << "'." << std::endl;
			return finish();
		}
	}
	if (config_YprOutput) {
		std::string file_name;
		if (config_YprOutputFile == "auto") {
			file_name = std::string(timestr) + std::string("_YPR.txt");
		}
		else {
			file_name = config_YprOutputFile;
		}
		ypr_file_out.open(file_name, std::ios::trunc);
		if (!ypr_file_out.is_open()) {
			std::cout << "Error opening file '" << file_name << "'." << std::endl;
			return finish();
		}
	}

	// Output socket stream.
	if (config_SocketOutput) {
		skt_server = new SocketServer(config_SocketOutputPort, 1, NonBlockingSocket);  // <Port>, <Max Connections>.
	}

    // Get first 100 samples, and measure their frequency.
    std::string input_line;
    int numSamples = 100;
    double delayAvgMs = 0.0;
    StartWinTimeCounter();

    while (input_readline(input_line) && numSamples > 0)
	{
        //printf("Input: b'%s'\n", std::string(input_line).erase(input_line.size() - 2).c_str());  // input_line.strip("\r\n").

		--numSamples;
        delayAvgMs += GetWinTimeMs();
        StartWinTimeCounter();
    }
    delayAvgMs /= 100;
    std::cout << "Average sampling frequency from board: " << 1000.0 / delayAvgMs << " Hz (" << (int)((uint8_t)ceil(delayAvgMs)) << " ms)" << std::endl;

    // Filter init.
	//razor_init(20);  // TODO put here an actual time measurement.
    razor_init((uint8_t)ceil(delayAvgMs));
    	
	while (input_readline(input_line))
	{
		//printf("Input: b'%s'\n", std::string(input_line).erase(input_line.size() - 2).c_str());  // input_line.strip("\r\n").

		std::vector<double> agm_input_vec;
		if (!parse_line(agm_input_vec, input_line)) {
			continue;  // Bad line, try the next one.
		}
		assert(agm_input_vec.size() == 9);

		std::string out_str;
		vector_to_str(out_str, agm_input_vec);
		//std::cout << out_str << std::endl;
		agmraw_file_out << out_str << std::endl;

		double agm_raw_arr[3][3];  // [Acc, Gyro, Mag] uncalibrated (biased) data.
		vector_to_array(agm_raw_arr, agm_input_vec);

		double agm_calib_arr[3][3];  // [Acc, Gyro, Mag] calibrated data.
		sensor_unbias_scale(agm_calib_arr, agm_raw_arr);

		array2d_to_str(out_str, agm_calib_arr);
		agmcalib_file_out << out_str << std::endl;

		double ypr_rad[3];  // [Yaw, Pitch, Roll] angles in radians.
		razor_heading(ypr_rad, agm_calib_arr);

		double ypr_deg[3];  // [Yaw, Pitch, Roll] angles in degrees.
		radians_to_degrees(ypr_deg, ypr_rad);

		vector_to_str(out_str, std::vector<double>(ypr_deg, ypr_deg+3));  // From array1d to vector to string.
		std::cout << out_str << std::endl;
		ypr_file_out << out_str << std::endl;

		// Output socket stream.
		if (config_SocketOutput) {
			if (skt_conn == NULL) {
				skt_conn = skt_server->Accept();
			}
			if (skt_conn != NULL) {
				skt_conn->SendLine(out_str);  // Warning: this modifies the string.
			}
		}
	}

    std::cout << std::endl << "No data to receive... c'est fini!" << std::endl;
	return finish();
}
