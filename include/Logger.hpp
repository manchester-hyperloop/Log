//
//  Logger.hpp
//  Logger
//
//  Created by Harry O'Brien on 19/10/2020.
//

#ifndef LIB_LOG_LOGGER_HPP
#define LIB_LOG_LOGGER_HPP

#include "Severity_Type.hpp"
#include "Log_File_Interface.hpp"

#ifndef UNIT_TEST
#include <RTClib.h>
#include <Arduino.h>
#else
#include <Arduino_Mock.hpp>
#include <RTC_Mock.hpp>
#endif

#ifdef ENABLE_ROS_OUTPUT
#include <Log_ROS.hpp>
#endif

/**
 * Logging interface
 */
class Logger
{
    /// Line no. of message to be logged.
    /// Incremented immediatly before writing to the output file.
    unsigned int log_line_number = -1;

    /// Compiled message exists here.
    String log_stream = "";

    /// Interface to the log file.
    Log_File_Interface *policy = nullptr;

    /// Real time clock.
    RTC_DS1307 rtc;

    /// Whether or not the logger is initialised.
    bool initialised = false;

    /// The severity type of the current log.
    severity_type current_log_severity;

    /// The ROS log manager.
#ifdef ENABLE_ROS_OUTPUT
    Log_ROS *log_ros;
#endif

public:
    /**
	 * Dissalow construction from another instance; Singleton class.
	 */
    Logger(Logger const &) = delete;

    /**
	 * Delete copy constructor; Singleton class.
	 */
    void operator=(Logger const &) = delete;

    /**
	 * Gets the singleton instance of the logger.
	 */
    static Logger &getLoggerInstance();

    /**
	 * Logger destructor. Closes log file interface on close.
	 */
    ~Logger();

    /**
	 * Initialise the logger.
	 */
    bool init();

    /**
	 * Appends the serverity type to the log stream and calls print_impl.
	 */
    template <severity_type severity, typename... Args>
    void print(Args... args);

    /**
	 * Gets the index of the last message written to the log file.
	 */
    unsigned int getLastMessageIndex() const;

    /**
	 * Gets the log interface that the logger is currently using.
	 */
    Log_File_Interface *getActiveLogInterface() const;

    /**
	 * Resets the logger instance. Only really used in testing.
	 */
    void reset();

private:
    /**
	 * Private constructor; singleton class.
	 */
    Logger() {}

    /**
	 * Gets the current time from the RTC.
	 */
    String get_time(bool dateAndTime);

    /**
	 * Generates the logline header.
	 */
    String get_logline_header();

    /**
	 * Generates the name for the log file.
	 */
    String generateLogName();

    /**
	 * Core printing functionality.
     * Called when `print_impl(First parm1, Rest... parm)` runs out of arguments
	 */
    inline void print_impl();

    /**
	 * Core printing functionality.
     * Recusively called until exhausts all arguments.
	 */
    template <typename First, typename... Rest>
    inline void print_impl(First parm1, Rest... parm);
};

template <severity_type severity, typename... Args>
void Logger::print(Args... args)
{
    current_log_severity = severity;

    switch (severity)
    {
    case severity_type::debug:
        log_stream += "<DEBUG>: ";
        break;
    case severity_type::warning:
        log_stream += "<WARNING>: ";
        break;
    case severity_type::error:
        log_stream += "<ERROR>: ";
        break;
    };
    
    print_impl(args...);
}

inline void Logger::print_impl()
{
    // Return if not initialised (prevents crash later when accessing policy)
    if (initialised)
        policy->write(get_logline_header() + log_stream);
    else
        Serial.println(String("(No init)") + log_stream);

    // Log to ROS
#ifdef ENABLE_ROS_OUTPUT
    log_ros -> Log(log_stream, current_log_severity);
#endif

    log_stream = "";
}

template <typename First, typename... Rest>
inline void Logger::print_impl(First parm1, Rest... parm)
{
    log_stream += String(parm1);
    print_impl(parm...);
}

#endif /* LIB_LOG_LOGGER_HPP */