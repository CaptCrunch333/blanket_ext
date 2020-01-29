#pragma once

#include "common_types.hpp"
#include "comm_device.hpp"
#include "BaseSerialComposer.hpp"
#include "BaseSerialParser.hpp"
#include "MsgEmitter.hpp"
#include "MsgReceiver.hpp"

class BaseCommunication : public msg_receiver, public msg_emitter
{

public:

	BaseCommunication(CommDevice*);
	void add_callback_msg_receiver(msg_receiver*);
	void receive_msg_data(DataMessage*);

private:

	BaseSerialParser* main_parser;
	BaseSerialComposer* main_composer;

	struct tHeader{
		uint16_t padding;
		uint8_t msgLen;
		uint8_t msgType;
	};

	struct tErrorCheck{
		uint16_t crc;
	};

	void send_packet(DataMessage*);

};