#include "comm_device.hpp"

CommDevice::CommDevice()
{
}

void CommDevice::receive_msg_data(DataMessage* t_msg)
{
    if (t_msg->getType() == msg_type::SERIALDATA)
    {
        send_data_to_hardware(t_msg);
    }
}