#include "BaseSerialParser.hpp"

// void BaseSerialParser::receive_msg_data(uint8_t data[],size_t len, msg_type _msg_type){
//     for (int i=0;i<len;i++){
//         if (this->state_machine(data[i])){
//             uint8_t* payload_data;
//             payload_data = data_stream + PacketProp.pad_len + PacketProp.hdr_len+1;// +1 for	EOH
// 			uint8_t _msg_type_num = (uint8_t)*(payload_data - 2);//-2 for EOH
// 			emit_message(payload_data,payload_len,static_cast<msg_type>(_msg_type_num));
//         }
//     }
// }

void BaseSerialParser::receive_msg_data(DataMessage* t_msg)
{
	if(t_msg->getType() == msg_type::SERIALDATA)
	{
		SerialDataMsg new_msg = *((SerialDataMsg*) t_msg);
		for (int i = 0; i < new_msg.len; i++)
		{
			if (this->state_machine(new_msg.data[i]))
			{
				uint8_t* payload_data;
				payload_data = data_stream + PacketProp.pad_len + PacketProp.hdr_len + 1; // +1 for	EOH
				uint8_t _msg_type_num = (uint8_t)*(payload_data - 2);
				msg_type t_type = (msg_type) _msg_type_num;
				if(t_type == msg_type::HEARTBEAT)
				{
					HeartbeatMsg temp_msg;
					//memcpy( ((uint8_t*) &temp_msg) + m_msg_handler.getMsgHeaderSize() , payload_data, payload_len);
					emit_message((DataMessage*) &temp_msg);
				}
			}
    	}
	}
}

//Frame anatomy:
//Padding | frame_len(byte) | msg_type | EOH | PAYLOAD | EOP | CRC16
uint8_t BaseSerialParser::state_machine(uint8_t input_byte){  //General Message:  |  Padding. (2-bytes)  |.  Msg. Len. (1-byte)  | Msg. type (1-byte)   |.  Payload (Variable)    |. Crc (2-bytes).   |
	uint8_t return_status = 0;
	switch (state)
	{
	case Padd_search: //$$$$$$$$$$$$$$$$ Reading Padding $$$$$$$$$$$$$$$$$$
		if (input_byte == packet_config.pad_data[byte_counter.padding_counter])
		{
			byte_counter.padding_counter++;
			data_stream[byte_index++] = input_byte;
		}
		else
		{
			byte_counter.padding_counter = 0;
			byte_index = 0; //Packet Faliure
		}
		if (byte_counter.padding_counter == packet_config.pad_len)
		{
			byte_counter.padding_counter = 0;
			state = header_read;
		}
		break;
	case header_read://$$$$$$$$$$$$$$$$$ Reading Header $$$$$$$$$$$$$$$$$$$
		data_stream[byte_index++] = input_byte;
		byte_counter.header_counter++;
		if (byte_counter.header_counter == packet_config.hdr_len)
		{
			byte_counter.header_counter = 0;
			state = EOH_check;
		}
		else //Because message length is the first byte
		{
			if ((uint16_t)input_byte <= packet_config.hdr_len + packet_config.pad_len + packet_config.crc_len){
				byte_counter.header_counter = 0;
				state = Padd_search;
				byte_index = 0;//Packet Faliure
			}
			else
			{
				payload_len = ((uint16_t)input_byte) - (packet_config.hdr_len + packet_config.pad_len + packet_config.crc_len + 2);//+2 For EOH and EOP
			}
		}
		break;
	case EOH_check:
		if (input_byte == packet_config.pad_EOH){
			data_stream[byte_index++] = input_byte;
			state = msg_read;
		}
		else
		{
			state = Padd_search;
			byte_index = 0;//Packet Faliure
		}
		break;
	case msg_read: //$$$$$$$$$$$$$$$$ Reading the message $$$$$$$$$$$$$$$$$$$$
		data_stream[byte_index++] = input_byte;
		byte_counter.msg_counter++;
		if (byte_counter.msg_counter == payload_len)
		{
			byte_counter.msg_counter = 0;
			state = EOP_check;
		}
		break;
	case EOP_check:
		if (input_byte == packet_config.pad_EOP){
			data_stream[byte_index++] = input_byte;
			state = crc_read;
		}
		else
		{
			state = Padd_search;
			byte_index = 0;//Packet Faliure
		}
		break;
	case crc_read://$$$$$$$$$$$$$$$$$ Reading the CRC $$$$$$$$$$$$$$$$$$$$$$
		data_stream[byte_index++] = input_byte;
		byte_counter.crc_counter++;
		if (byte_counter.crc_counter == packet_config.crc_len)
		{
			byte_counter.crc_counter = 0;
			state = done_reading;
		}
		break;
	}
	if (state == done_reading){
		uint16_t crc_res = crc::crc16_ccitt(data_stream, byte_index, 0);
		if (crc_res == 0) {
			return_status = 1;
		}
		state = Padd_search;
		byte_index = 0;
	}
	return return_status;
}
void BaseSerialParser::configure(tPacketProp packet_config_para)
{
	packet_config = packet_config_para;
	byte_index = 0;
}