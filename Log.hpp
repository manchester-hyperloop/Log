//
//  Log.hpp
//  Logger
//
//  Created by Harry O'Brien on 17/10/2020.
//

#ifndef LIB_LOG_LOG_HPP
#define LIB_LOG_LOG_HPP

#include "Logger.hpp"
#include "Severity_Type.hpp"

/**
 * The singleton instance of the logger
 * Declared as extern to be accessable everywhere in the program
 */
extern Logger &log_inst;

#ifdef LOGGING_LEVEL_1
#define LOG log_inst.print<severity_type::debug>
#define LOG_ERR log_inst.print<severity_type::error>
#define LOG_WARN log_inst.print<severity_type::warning>
#else
#define LOG(...)
#define LOG_ERR(...)
#define LOG_WARN(...)
#endif

#ifdef LOGGING_LEVEL_2
#define ELOG log_inst.print<severity_type::debug>
#define ELOG_ERR log_inst.print<severity_type::error>
#define ELOG_WARN log_inst.print<severity_type::warning>
#else
#define ELOG(...)
#define ELOG_ERR(...)
#define ELOG_WARN(...)
#endif

#endif /* LIB_LOG_LOG_HPP */