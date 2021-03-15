//
//  Logger.cpp
//  Logger
//
//  Created by Harry O'Brien on 22/11/2020.
//

#include "Logger.hpp"

bool Logger::init()
{
    if (initialised)
        return initialised;

    if (!rtc.begin())
    {
        Serial.println("Couldn't find RTC...");
        return false;
    }

    policy = new Log_File_Interface;

    // Check that policy was created successfully (not nullptr)
    if (!policy)
    {
        Serial.println(F("LOGGER: Unable to create the logger instance"));
        return false;
    }

    String name = generateLogName();
    initialised = policy->open_ostream(name);

    return initialised;
}

Logger &Logger::getLoggerInstance()
{
    static Logger instance;
    return instance;
}

Logger::~Logger()
{
    if (policy)
        delete policy;
}

String Logger::get_time(bool dateAndTime)
{
    DateTime time = rtc.now();
    if (dateAndTime)
        return time.timestamp(DateTime::TIMESTAMP_FULL);
    else
        return time.timestamp(DateTime::TIMESTAMP_TIME);
}

String Logger::get_logline_header()
{
    String header = "";

    header += String(++log_line_number) + " < " + get_time(false) + " - ";

    header += String(micros()) + " > ~ ";

    return header;
}

String Logger::generateLogName()
{
    String time = get_time(true);
    time.replace(':', '.');
    return time + ".log";
}

unsigned int Logger::getLastMessageIndex() const
{
    return log_line_number;
}

Log_File_Interface *Logger::getActiveLogInterface() const
{
    return policy;
}

void Logger::reset()
{
    Serial.println(F("WARNING: Resetting Logger!"));
    initialised = false;
    delete policy;
    policy = nullptr;
}