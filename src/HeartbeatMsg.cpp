#include "HeartbeatMsg.hpp"

msg_type HeartbeatMsg::getType()
{
	return msg_type::HEARTBEAT;
}

const int HeartbeatMsg::getSize()
{
	return sizeof(HeartbeatMsg);
}