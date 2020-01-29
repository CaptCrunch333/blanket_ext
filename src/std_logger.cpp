#include "std_logger.hpp"

StdLogger::StdLogger()
{

}

StdLogger::~StdLogger()
{
    m_log_file.close();
}

void StdLogger::log(const char* t_log, LoggerLevel t_log_level){
    printLogLevel(t_log_level);
    printf(t_log);
    printf("\n");
}

void StdLogger::log(const char* t_log, float t_para_1, LoggerLevel t_log_level){
    printLogLevel(t_log_level);
    printf(t_log, t_para_1);
    printf("\n");
}

void StdLogger::log(const char* t_log, float t_para_1, float t_para_2, LoggerLevel t_log_level){
    printLogLevel(t_log_level);
    printf(t_log, t_para_1, t_para_2);
    printf("\n");
}

void StdLogger::log(const char* t_log, float t_para_1 , float t_para_2, float t_para_3, LoggerLevel t_log_level){
    printLogLevel(t_log_level);
	printf(t_log, t_para_1, t_para_2,t_para_3);
    printf("\n");
}

void StdLogger::enableFileLog(LoggerLevel t_loggerlevel)
{
    file_logger_level = t_loggerlevel;
    m_file_logging_enabled = true;
    string t_FileName = "/home/ffuav01/logs/log_";
    m_time = time(NULL);
    m_sTm = gmtime(&m_time);
    strftime(m_buff, sizeof(m_buff), "%Y-%m-%d %H:%M:%S", m_sTm);
    for(int i = 0; i<sizeof(m_buff); i++)
    {
        t_FileName.push_back(m_buff[i]);
    }
    t_FileName.append(".csv");
    m_log_file.open(t_FileName);
}

void StdLogger::disableFileLog()
{
    m_file_logging_enabled = false;
    m_log_file.close();
}

void StdLogger::printLogLevel(LoggerLevel t_log_level){
    switch (t_log_level){
        case LoggerLevel::Info:
        {
			printf("Info: ");
            break;
        }
        case LoggerLevel::Warning:
        {
			printf("Warn: ");
            break;
        }
        case LoggerLevel::Error:
        {
			printf("Error: ");
            break;
        }

    }
}

void StdLogger::logtofile(const char* t_log, float t_para_1, LoggerLevel t_log_level)
{
    if(m_file_logging_enabled)
    {
        if((int) file_logger_level <= (int) t_log_level)
        {
            m_log_file << t_log << "," << t_para_1 << endl;
        }
    }
}

void StdLogger::logtofile(const char* t_log, float t_para_1, float t_para_2, LoggerLevel t_log_level)
{
    if(m_file_logging_enabled)
    {
        if((int) file_logger_level <= (int) t_log_level)
        {
            m_log_file << t_log << "," << t_para_1 << "," << t_para_2 << endl;
        }
    }
}

void StdLogger::logtofile(const char* t_log, float t_para_1 , float t_para_2, float t_para_3, LoggerLevel t_log_level)
{
    if(m_file_logging_enabled)
    {
        if((int) file_logger_level <= (int) t_log_level)
        {
            m_log_file << t_log << "," << t_para_1 << "," << t_para_2 << "," << t_para_3 << endl;
        }
    }
}