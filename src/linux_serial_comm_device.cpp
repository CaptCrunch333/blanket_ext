#include "linux_serial_comm_device.hpp"

//TODO DELETE
#include "Timer.hpp"

LinuxSerialCommDevice* LinuxSerialCommDevice::A;
int LinuxSerialCommDevice::m_serial_port;
SerialDataMsg LinuxSerialCommDevice::m_serial_data;

LinuxSerialCommDevice::LinuxSerialCommDevice()
{

}

LinuxSerialCommDevice::~LinuxSerialCommDevice()
{
    pthread_join(this->m_thread_id, NULL);
}

bool LinuxSerialCommDevice::attach_hardware_sender(void* sender_obj)
{
    m_dev_path = (string*) sender_obj;
    m_serial_port = open(m_dev_path->c_str(), O_RDWR, O_NOCTTY);
    if (m_serial_port == -1)
    {
        perror("open_port: Unable to open port ");
        std::cout << m_dev_path->c_str() << std::endl;
        return false;
        //exit(1);
    }
    // //***** Enabling interrupts *****//
    // m_incoming_signal_handler.sa_handler = receive_data_from_hardware;
    // m_incoming_signal_handler.sa_flags = 0;
    // m_incoming_signal_handler.sa_restorer = NULL; 
    // sigaction(SIGIO,&m_incoming_signal_handler,NULL);

    // fcntl(m_serial_port, F_SETFL, FNDELAY);
    // fcntl(m_serial_port, F_SETOWN, getpid());
    // fcntl(m_serial_port, F_SETFL,  O_ASYNC );
    // //******************************//

    tcgetattr(m_serial_port, &m_IO_handler); //get parameters assciated with port

    // Set in/out baud rate to be 115200
    cfsetispeed(&m_IO_handler, B1152000);
    cfsetospeed(&m_IO_handler, B1152000);

    m_IO_handler.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
    m_IO_handler.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
    m_IO_handler.c_cflag &= ~CSIZE;
    m_IO_handler.c_cflag |= CS8; // 8 bits per byte (most common)
    m_IO_handler.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)
    m_IO_handler.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)

    m_IO_handler.c_lflag &= ~ICANON;
    m_IO_handler.c_lflag &= ~ECHO; // Disable echo
    m_IO_handler.c_lflag &= ~ECHOE; // Disable erasure
    m_IO_handler.c_lflag &= ~ECHONL; // Disable new-line echo
    //m_IO_handler.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
    m_IO_handler.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    m_IO_handler.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

    m_IO_handler.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    m_IO_handler.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
    //m_IO_handler.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
    //m_IO_handler.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

    // Read bytes. The behaviour of read() (e.g. does it block?,
    // how long does it block for?) depends on the configuration
    // settings above, specifically VMIN and VTIME
    m_IO_handler.c_cc[VTIME] = 0;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    m_IO_handler.c_cc[VMIN] = 0;

    // Save tty settings, also checking for error
    if (tcsetattr(m_serial_port, TCSANOW, &m_IO_handler) != 0)
    {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
        return false;
    }
    A = this;
    pthread_create(&(A->m_thread_id), NULL, A->checkReceiver, NULL);
    return true;
}

void LinuxSerialCommDevice::reset_hardware(void* sender_obj)
{
    close(m_serial_port);
    this->attach_hardware_sender(sender_obj);
}

void LinuxSerialCommDevice::receive_data_from_hardware(int t_status)
{
    m_serial_data.len = read(m_serial_port, &(m_serial_data.data), 1024);
    if (m_serial_data.len < 0) 
    {
        printf("Error reading: %s", strerror(errno));
    }
    else
    {
        A->emit_message(&m_serial_data);
    }
}

void LinuxSerialCommDevice::attach_hardware_callback(void* call_back_obj)
{

}

void LinuxSerialCommDevice::receive_msg_data(DataMessage* t_msg)
{
    send_data_to_hardware(t_msg);
}

void LinuxSerialCommDevice::send_data_to_hardware(DataMessage* t_msg)
{
    //uint8_t buff[1];
    if(t_msg->getType() == msg_type::SERIALDATA)
    {
        SerialDataMsg* data = (SerialDataMsg*) t_msg;
        write(m_serial_port, data->data, data->len);
    }
}

void* LinuxSerialCommDevice::checkReceiver(void*)
{
    //"NON BUSY" Sleep for 1 ms 
    while(1)
    {
        usleep(1000);
        m_serial_data.len = read(m_serial_port, &(m_serial_data.data), 1024);
        if (m_serial_data.len > 0) 
        {
            //std::cout <<"data received/n";
            A->emit_message((DataMessage*) &m_serial_data);
        }
    }
}