#pragma once

#include <cstdint>
class crc
{
	private:

		static const uint16_t crc16tab[256];

	public:
	
		static unsigned short crc16_ccitt(uint8_t *, int, unsigned short); // crc: crc seed to start with. Use 0x0000
};