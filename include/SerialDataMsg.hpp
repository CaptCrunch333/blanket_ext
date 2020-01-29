#pragma once

#include <cstdint>
#include "DataMessage.hpp"

class SerialDataMsg : public DataMessage
{

public:

    msg_type getType();
    const int getSize();

    uint8_t data[1024];
    int len;
};