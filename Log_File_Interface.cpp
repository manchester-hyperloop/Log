//
//  Log_Policy.cpp
//  Logger
//
//  Created by Harry O'Brien on 17/10/2020.
//

#include "Log_File_Interface.hpp"

bool Log_File_Interface::open_ostream(const String &file_name)
{
    if (!sd.begin(CS_Pin))
    {
        Serial.println(F("LOGGER: Unable to initialise SD card..."));
        return false;
    }

    String dir_name = "Logs";
    sd.mkdir(dir_name.c_str());

    String const logPath = dir_name + "/" + file_name;

    log_file = sd.open(logPath, FILE_WRITE);
    if (!log_file)
    {
        Serial.println(F("Logger: Unable to open an output stream!"));
        return false;
    }

    return true;
}

void Log_File_Interface::close_ostream()
{
    if (log_file)
    {
        log_file.close();
    }
}

void Log_File_Interface::write(const String &msg)
{
    log_file.println(msg);
#ifdef LOG_TO_SERIAL
    Serial.println(msg);
#endif
    log_file.flush();
}

Log_File_Interface::~Log_File_Interface()
{
    if (log_file)
    {
        close_ostream();
    }
}

File &Log_File_Interface::getLogFile()
{
    return log_file;
}