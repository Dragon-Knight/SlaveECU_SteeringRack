#pragma once
#include <inttypes.h>

/*
	Класс работы с датчиком угла поворота руля.
	Модель: 479452659R
*/

class SteeringAngleSensor
{
	static constexpr uint16_t PACKET_ID = 0x00C2;
	static constexpr uint8_t PACKET_LENGTH = 7;
	
	struct __attribute__((packed)) sensor_packet_t
	{
		uint8_t crc;
		uint8_t _null;
		uint8_t counter : 4;
		uint8_t error : 4;
		int16_t roll;
		int16_t angle;
	};
	
	struct sensor_t
	{
		float angle;		// Улог поворота в гладусах
		float roll;			// Скорость вращения в Драконах
		float dt;			// Дельта пакетов в секундах
	};
	
	public:
		SteeringAngleSensor() : _last_time(0), _last_count(255)
		{}
		
		bool PutPacket(uint32_t time, uint16_t id, uint8_t *raw, uint8_t length)
		{
			if(id != PACKET_ID) return false;
			if(length != PACKET_LENGTH) return false;
			
			reverse_memcpy(_raw_array, raw, length);
			// Массив _raw_array кастится на объект _obj, поэтому можно сразу работать с объектом
			
			if(_CalculateCRC() != _obj->crc) return false;
			if(_obj->error != 0) return false;
			if(_obj->counter == _last_count) return false;
			
			_obj->angle -= 0x8000;
			_obj->roll -= 0x8000;
			
			_sensor_data_float.angle = (float)_obj->angle / 10.0f;
			_sensor_data_float.roll = (float)_obj->roll / 10.0f;
			_sensor_data_float.dt = (float)(time - _last_time) / 1000.0f;
			
			_last_time = time;
			_last_count = _obj->counter;
			
			return true;
		}
		
		const sensor_packet_t *data_int = (sensor_packet_t *) _raw_array;
		const sensor_t *data_float = (sensor_t *) &_sensor_data_float;
		
	private:
		uint8_t _CalculateCRC()
		{
			uint8_t crc = 0xFF;
			for(uint8_t i = 1; i < PACKET_LENGTH; ++i)
			{
				crc -= _raw_array[i];
			}
			return crc;
		}
		
		void reverse_memcpy(uint8_t *dest, const uint8_t *src, uint16_t len)
		{
			if(len == 0) return;
			
			const uint8_t *src_end = src + len - 1;
			while(len--)
				*dest++ = *src_end--;

			return;
		}
		
		uint32_t _last_time;
		uint8_t _last_count;
		
		uint8_t _raw_array[PACKET_LENGTH];
		sensor_packet_t *_obj = (sensor_packet_t *) _raw_array;
		sensor_t _sensor_data_float;
};
