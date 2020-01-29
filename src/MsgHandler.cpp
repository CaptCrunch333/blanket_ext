#include "MsgHandler.hpp"

MsgHandler::MsgHandler()
{
    m_msg_addr_len = sizeof(DataMessage);
}

void MsgHandler::setMsg(DataMessage* t_msg)
{
    m_msg = t_msg;
}

uint8_t MsgHandler::getType()
{
    return (uint8_t) m_msg->getType();
}

uint8_t* MsgHandler::Serialize()
{
    uint8_t* t_msg_payload = (uint8_t*) m_msg;
    t_msg_payload = t_msg_payload + m_msg_addr_len;
    return t_msg_payload;
}

int MsgHandler::getMsgHeaderSize()
{
    return m_msg_addr_len;
}

int MsgHandler::getSerialLen()
{
    return (m_msg->getSize() - m_msg_addr_len);
}