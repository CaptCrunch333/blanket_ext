#pragma once
#include "DataMessage.hpp"

class ControllerOutputMsg : public DataMessage
{

public:

    msg_type getType();
    const int getSize();
    void setControlSignal(float, control_system);
    float getControlSignal(void);
    control_system getSource(void);

private:
    
    float m_control_signal;
    control_system m_from_system;  
};