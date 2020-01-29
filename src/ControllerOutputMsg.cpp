#include "ControllerOutputMsg.hpp"

msg_type ControllerOutputMsg::getType()
{
    return msg_type::CONTROLLEROUTPUTMSG;
}

const int ControllerOutputMsg::getSize()
{
    return sizeof(ControllerOutputMsg);
}

void ControllerOutputMsg::setControlSignal(float t_data, control_system t_from_system)
{
    m_control_signal = t_data;
    m_from_system = t_from_system;
}

float ControllerOutputMsg::getControlSignal(void)
{
    return m_control_signal;
}

control_system ControllerOutputMsg::getSource(void)
{
    return m_from_system;
}