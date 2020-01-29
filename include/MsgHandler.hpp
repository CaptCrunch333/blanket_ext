#pragma once
#include "DataMessage.hpp"

class MsgHandler{

    public:

        MsgHandler();
        void setMsg(DataMessage*);
        uint8_t* Serialize();
        uint8_t getType();
        int getSerialLen();
        int getMsgHeaderSize();

    private:

        int m_msg_addr_len;
        DataMessage* m_msg;

};