#pragma once
#include <inttypes.h>

class SteeringAngleSensorBase
{
	public:
		
		enum error_t : uint8_t
		{
			ERROR_NONE,
			ERROR_ID,
			ERROR_LENGTH,
			ERROR_CRC,
			ERROR_ANGLE,
			ERROR_ERR_SENSOR,
			ERROR_COUNTER,
			ERROR_LOST
		};

	protected:
		
		static constexpr uint16_t PACKET_ID = 0x00C2;
		static constexpr uint8_t PACKET_LENGTH = 7;
		static constexpr uint32_t LOST_IDLE_TIME = 50;
		static constexpr uint32_t LOST_START_TIME = LOST_IDLE_TIME * 30;

		// Сырой CAN пакет данных от датчика в обратном порядке байт
		struct __attribute__((packed)) sensor_packet_t
		{
			uint8_t crc;
			uint8_t _null;
			uint8_t counter : 4;
			uint8_t error : 4;
			int16_t roll;		// 0.1 чего-то
			int16_t angle;		// 0.1 градуса
		};

		struct sensor_t
		{
			float angle;		// Улог поворота в гладусах
			float roll;			// Скорость вращения в Драконах
			float dt;			// Дельта пакетов в секундах
		};
};
