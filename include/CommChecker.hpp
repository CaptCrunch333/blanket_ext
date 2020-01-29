#pragma once

#include "MsgEmitter.hpp"
#include "comm_device.hpp"
#include "Timer.hpp"
#include "logger.hpp"
#include "TimedBlock.hpp"
#include "HeartbeatMsg.hpp"

class CommChecker : public msg_emitter, public msg_receiver, public TimedBlock
{

public:

    CommChecker(CommDevice*, void*, block_frequency);
    ~CommChecker();

    void sendRequest(DataMessage*);
    void receive_msg_data(DataMessage*);
    void loopInternal();

private:

    const int m_timeout = 5000000;
    Timer m_timer;
    CommDevice* m_dev;
    void* m_comm_port;
};