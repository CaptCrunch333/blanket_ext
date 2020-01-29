#pragma once
/**
* @file std_logger.hpp
*
* @brief Logging through STL
*
* @ingroup UAVFirmware
* (Note: this needs exactly one @defgroup somewhere)
*
* @author Mohamad Chehadeh
* Contact: mohamad.chehadeh@ku.ac.ae
*
* @version 1.0 26/11/2019
*
**/
#include "logger.hpp"
#include <stdio.h>
#include <fstream>
#include <time.h>

using namespace std;

class StdLogger : public Logger
{
    private:

        bool m_file_logging_enabled = false;
        ofstream m_log_file;
        LoggerLevel file_logger_level=LoggerLevel::Info;
        char m_buff[20];
        struct tm *m_sTm;
        time_t m_time;

        void printLogLevel(LoggerLevel);

    public:

        StdLogger();
        ~StdLogger();
        void log(const char*,LoggerLevel);
        void log(const char*,float,LoggerLevel);
        void log(const char*,float,float,LoggerLevel);
        void log(const char*,float,float,float,LoggerLevel);

        void logtofile(const char*,float,LoggerLevel);
        void logtofile(const char*,float,float,LoggerLevel);
        void logtofile(const char*,float,float,float,LoggerLevel);

        void enableFileLog(LoggerLevel);// Example: if LoggerLevel::Warning is passed only LoggerLevel::Warning and LoggerLevel::Error will be written to file
        void disableFileLog();
};