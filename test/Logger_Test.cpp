//
//  Logger_Test.cpp
//  Hyperloop_Embedded
//
//  Created by Harry O'Brien on 15/10/2020.
//  Copyright © 2020 Hyperloop Manchester. All rights reserved.
//

#include <unity.h>
#include <Log.hpp>
#include <string>
#include <sstream>
#include <iostream>

#include <SD_Mock.hpp>
#include <RTC_Mock.hpp>
#include <Arduino_Mock.hpp>

/****************************************************************************************************/
/*                                             HELPERS                                              */
/****************************************************************************************************/
// Get the log policy from the log instance
File &getLogFileMock()
{
    Log_File_Interface *policy = log_inst.getActiveLogInterface();
    TEST_ASSERT(policy);

    return policy->getLogFile();
}

std::string getLastLoggedMessage()
{
    return getLogFileMock().getLastMessage();
}

/****************************************************************************************************/
/*                                               TESTS                                              */
/****************************************************************************************************/

void test_message_index_starts_at_0(void)
{
    // Initially, index should be -1 (no messages have been logged yet!)
    TEST_ASSERT(log_inst.getLastMessageIndex() == -1);

    // Log a message
    LOG("Testing 123");

    // Last index written should now be 0
    TEST_ASSERT(log_inst.getLastMessageIndex() == 0);
}

// logger should print index of each log message
void test_log_message_index(void)
{
    // Get last index written
    unsigned long const lastIndex = log_inst.getLastMessageIndex();

    // Log a message
    LOG("test_log_message_index");

    // Index should now be 1 more than previous
    TEST_ASSERT(log_inst.getLastMessageIndex() == lastIndex + 1);

    // output message should hate
    TEST_ASSERT(getLastLoggedMessage()[0] == '1');
}

// logger should save the message passed to it to SD
void test_logger_should_save_message_to_sd(void)
{
    // Log a message
    std::string message = "test_logger_should_save_message_to_sd";
    LOG(message);
    TEST_ASSERT(getLastLoggedMessage().find(message));
}

// logger should be able to take variadic arguments of different types and print all of them
void test_logger_takes_variadic_args(void)
{
    std::string const message1 = "my testy boi val is: ";
    unsigned int test_val = 2334;
    std::string const message2 = " and some bool is: ";
    bool test_bool = false;

    LOG(message1, test_val, message2, test_bool);

    std::string compiledMessage = message1 + patch::to_string(test_val) + message2 + patch::to_string(test_bool);

    TEST_ASSERT(getLastLoggedMessage().find(compiledMessage) != std::string::npos);
}

// logger should print time with each message
void test_logger_prints_time(void)
{
    // Log a message
    LOG("test_logger_prints_time");

    // Get the 'time'
    RTC_DS1307 rtc_mock;
    std::string timeNow = rtc_mock.now().timestamp(DateTime::timestampOpt::TIMESTAMP_TIME);

    TEST_ASSERT(getLastLoggedMessage().find(timeNow) != std::string::npos);
}

// logger should print either, INFO, ERROR, or WARNING with each message depending on severity
void test_logger_prints_severity(void)
{
    // test for debug
    LOG("test_logger_prints_severity (debug)");
    TEST_ASSERT(getLastLoggedMessage().find("DEBUG") != std::string::npos);

    LOG_WARN("test_logger_prints_severity (warn)");
    TEST_ASSERT(getLastLoggedMessage().find("WARNING") != std::string::npos);

    LOG_ERR("test_logger_prints_severity (err)");
    TEST_ASSERT(getLastLoggedMessage().find("ERROR") != std::string::npos);
}

// Logger should be able to reset itself
void test_logger_may_reset(void)
{
    // Check that policy is still alive
    TEST_ASSERT(log_inst.getActiveLogInterface() != nullptr);

    // reset
    log_inst.reset();

    // policy should now be null
    TEST_ASSERT(log_inst.getActiveLogInterface() == nullptr);
}

// Logger should fail to start and return false if unable to initialise SD card hardware
void test_logger_fails_on_sd_failure(void)
{
    // Reset the log and force the sd card to not initialise properly
    log_inst.reset();
    SdFat::canInitialise = false;

    // Test that the init function fails
    TEST_ASSERT_FALSE(log_inst.init());

    // Restore the sd card mock to it's normal state
    SdFat::canInitialise = true;
}

// Logger should fail to start and return false if unable to open an output stream
void test_logger_fails_on_output_failure(void)
{
    // Reset the log and force the file to not open properly
    log_inst.reset();
    File::file_is_open = false;

    // Test that the init function fails
    TEST_ASSERT_FALSE(log_inst.init());

    // Restore the file mock to it's normal state
    File::file_is_open = true;
}

// Logger should fail to start and return fals if unable to start real-time clock (RTC)
void test_logger_fails_on_rtc_failure(void)
{
    // Reset the log and force the real-time clock to not initialise properly
    log_inst.reset();
    RTC_DS1307::canInitialise = false;

    // Test that the init function fails
    TEST_ASSERT_FALSE(log_inst.init());

    // Restore the rtc mock to it's normal state
    RTC_DS1307::canInitialise = true;
}

int main(int argc, char **argv)
{
    UNITY_BEGIN();

    log_inst.init();

    RUN_TEST(test_message_index_starts_at_0);
    RUN_TEST(test_log_message_index);
    RUN_TEST(test_logger_should_save_message_to_sd);
    RUN_TEST(test_logger_takes_variadic_args);
    RUN_TEST(test_logger_prints_time);
    RUN_TEST(test_logger_prints_severity);

    RUN_TEST(test_logger_may_reset);
    RUN_TEST(test_logger_fails_on_sd_failure);
    RUN_TEST(test_logger_fails_on_output_failure);
    RUN_TEST(test_logger_fails_on_rtc_failure);

    UNITY_END();
}