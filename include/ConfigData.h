#pragma once
#include <inttypes.h>

namespace Config
{
	struct __attribute__((packed)) eeprom_body_t
	{
		struct
		{
			bool enable = true;
			bool invert = false;
			int16_t offset = 0;
		} rack1;
		struct
		{
			bool enable = true;
			bool invert = true;
			int16_t offset = 0;
		} rack2;
	} config;
};

// Должна быть выровнена строго кратно 26 байтам.
// Избегать чтобы переменные пересекали кратность 26 байт
