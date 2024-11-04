#pragma once
#include <inttypes.h>

namespace Config
{
	struct __attribute__((packed)) eeprom_body_t
	{
		struct
		{
			bool enable = false;
		} rack1;
		struct
		{
			bool enable = false;
		} rack2;
	};
};
