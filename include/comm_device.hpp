#pragma once

#include "MsgReceiver.hpp"
#include "MsgEmitter.hpp"
#include "SerialDataMsg.hpp"

using namespace std;

class CommDevice : public msg_emitter, public msg_receiver
{
    public:

        CommDevice();
        virtual void receive_msg_data(DataMessage* t_msg) override;
        virtual void attach_hardware_callback(void* call_back_obj) = 0;
        virtual void reset_hardware(void* sender_obj) = 0;
        virtual bool attach_hardware_sender(void* sender_obj) = 0;
        virtual void send_data_to_hardware(DataMessage* t_msg) = 0;
    
    protected:
    
        list<msg_receiver*> message_parsers;

    private:

        msg_type _msg_type;
};