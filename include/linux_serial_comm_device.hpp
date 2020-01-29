#pragma once

// C library headers
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <boost/asio.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
// Linux headers
#include <pthread.h>
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/signal.h>
#include <netinet/in.h>
#include <csignal>


#include "comm_device.hpp"
#include "SerialDataMsg.hpp"

class LinuxSerialCommDevice : public CommDevice
{
    public:

        static int m_serial_port;
        string* m_dev_path;

        LinuxSerialCommDevice();
        ~LinuxSerialCommDevice();
        bool attach_hardware_sender(void* sender_obj) override;
        void attach_hardware_callback(void* call_back_obj) override;
        void reset_hardware(void* sender_obj);
        void send_data_to_hardware(DataMessage* ) override;
        void receive_msg_data(DataMessage*);
        void receive_data_from_hardware(int);

    private:
        //TODO: make a static tracker wrapper
        static LinuxSerialCommDevice* A;
        pthread_t m_thread_id;
        //TODO: is this ok const?
        const int max_msg_size = 1024;
        struct termios m_IO_handler;
        //struct sigaction m_incoming_signal_handler;
        static SerialDataMsg m_serial_data;

        void monitorUserInterrupt();
        static void *checkReceiver(void*);
};