#include "SerialDataMsg.hpp"

msg_type SerialDataMsg::getType()
{
    return msg_type::SERIALDATA;
}

const int SerialDataMsg::getSize()
{
    return sizeof(SerialDataMsg);
}