#pragma once 

#include "DataMessage.hpp"

class HeartbeatMsg : public DataMessage
{
public:

	msg_type getType();
	const int getSize();
	int data = 1;
};