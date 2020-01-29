#pragma once

#include <string.h> // memcpy
#include "MsgReceiver.hpp"
#include "MsgEmitter.hpp"
#include "CRC.hpp"
#include "SerialDataMsg.hpp"
#include "MsgHandler.hpp"

class BaseSerialComposer : public msg_emitter
{

	public:

		//DataMessage* get_tx_frame_buf();
		//void configure(int para_padding_len, int para_header_len, int para_crc_len, uint8_t para_padding);
		void configure(tPacketProp);
		void send_packet(DataMessage*);

	private:

	int msg_len;
	int padding_len;
	int header_len;
	int crc_len;
	uint8_t padding;
	//uint8_t frame_bytes[1024];
	SerialDataMsg* frame_bytes = new SerialDataMsg;
	tPacketProp packet_config;
	MsgHandler m_msg_handler;
};