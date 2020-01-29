#include "BaseCommunication.hpp"

BaseCommunication::BaseCommunication(CommDevice* comm_device_para)
{
	main_parser = new BaseSerialParser();
	main_composer = new BaseSerialComposer();
    comm_device_para->add_callback_msg_receiver((msg_receiver*)main_parser);
	main_parser->configure(PacketProp);
	main_composer->configure(PacketProp);
	main_composer->add_callback_msg_receiver((msg_receiver*)comm_device_para);
}

void BaseCommunication::add_callback_msg_receiver(msg_receiver* _msg_receiver)
{
	//Removed, dublication of msg senders
	//msg_emitter::add_callback_msg_receiver(_msg_receiver);
	main_parser->add_callback_msg_receiver(_msg_receiver);
}

//TODO: make CRC more efficient by doing it for payload only
void BaseCommunication::receive_msg_data(DataMessage* t_msg)
{
	if ((int)t_msg->getType() < (int)msg_type::internal_msg_start)
	{
		this->send_packet(t_msg);
	}
}

void BaseCommunication::send_packet(DataMessage* t_msg)
{
	main_composer->send_packet(t_msg);
}