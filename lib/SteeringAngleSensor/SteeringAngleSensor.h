#pragma once
#include <inttypes.h>
#include "SteeringAngleSensorBase.h"

/*
	Класс работы с датчиком угла поворота руля.
	Модель: 479452659R
*/

template <uint8_t _idx = 0> 
class SteeringAngleSensor : public SteeringAngleSensorBase
{
	using callback_error_t = void (*)(uint8_t idx, error_t code);
	
	public:
		
		SteeringAngleSensor(callback_error_t error) : 
			_callback_error(error), _last_time(0), _last_count(255),
			_error(ERROR_NONE), _error_last(ERROR_NONE)
		{}
		
		bool PutPacket(uint32_t &time, uint16_t id, uint8_t *raw, uint8_t length)
		{
			if(id != PACKET_ID){ _error = ERROR_ID; return false; }
			if(length != PACKET_LENGTH){ _error = ERROR_LENGTH; return false; }
			
			reverse_memcpy(_raw_array, raw, length);
			// Массив _raw_array кастится на объект _raw_obj, поэтому можно сразу работать с объектом
			
			if(_CalculateCRC() != _raw_obj->crc){ _error = ERROR_CRC; return false; }
			if(_raw_obj->angle == 0xFFFF){ _error = ERROR_ANGLE; return false; }
			if(_raw_obj->error != 0){ _error = ERROR_ERR_SENSOR; return false; }
			if(_raw_obj->counter == _last_count){ _error = ERROR_COUNTER; return false; }
			
			_raw_obj->angle -= 0x8000;
			_raw_obj->roll -= 0x8000;
			
			_sensor_data_float.angle = (float)_raw_obj->angle / 10.0f;
			_sensor_data_float.roll = (float)_raw_obj->roll / 10.0f;
			_sensor_data_float.dt = (float)(time - _last_time) / 1000.0f;
			
			_last_time = time;
			_last_count = _raw_obj->counter;
			
			_error = ERROR_NONE;
			return true;
		}
		
		void Tick(uint32_t &time)
		{
			if(time - _last_time > LOST_IDLE_TIME)
			{
				_error = ERROR_LOST;
			}
			
			if(_error != _error_last)
			{
				_error_last = _error;
				
				_callback_error(_idx, _error);
			}
			
			return;
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
		
		callback_error_t _callback_error;
		
		uint32_t _last_time;
		uint8_t _last_count;
		
		uint8_t _raw_array[PACKET_LENGTH];
		sensor_packet_t *_raw_obj = (sensor_packet_t *) _raw_array;
		sensor_t _sensor_data_float;

		error_t _error;
		error_t _error_last;
};
