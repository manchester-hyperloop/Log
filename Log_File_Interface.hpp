//
//  Log_Policy.h
//  Logger
//
//  Created by Harry O'Brien on 17/10/2020.
//

#ifndef LIB_LOG_LOG_POLICY_H
#define LIB_LOG_LOG_POLICY_H

#ifndef UNIT_TEST
#include <Arduino.h>
#include <SdFat.h>
#else
#include <Arduino_Mock.hpp>
#include <SD_Mock.hpp>
#endif

/*
 * Implementation which allow to write into a file
 */

class Log_File_Interface
{
	/// File system object.
	SdFat sd;

	/// Log file.
	File log_file;

	/// CS Pin pin mapping.
	uint8_t const CS_Pin = 10;

public:
	/**
	 * Opens output file stream
	 * @param file_name name of output file to open
	 * @return true if successful, false otherwise
	 */
	bool open_ostream(const String &file_name);

	/**
	 * Close the file output stream
	 */
	void close_ostream();

	/**
	 * Write text to the output file. Prints to serial if LOG_TO_SERIAL is defined.
	 * @param msg text to output to file.
	 */
	void write(const String &msg);

	/**
	 * Closes output file if open
	 */
	~Log_File_Interface();

	/**
	 * Gets the currently open log file
	 */
	File &getLogFile();
};

#endif /* LIB_LOG_LOG_POLICY_H */
