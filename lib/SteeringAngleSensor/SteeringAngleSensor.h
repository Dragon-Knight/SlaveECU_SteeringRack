#pragma once
#include <inttypes.h>
#include <CUtils.h>
#include "SteeringAngleSensorBase.h"

/*
	Класс работы с датчиком угла поворота руля.
	Модель: 479452659R
*/

class SteeringAngleSensor : public SteeringAngleSensorBase
{
	using callback_error_t = void (*)(uint8_t idx, error_t code);
	
	public:
		
		SteeringAngleSensor(uint8_t idx, callback_error_t error) : 
			_idx(idx), _callback_error(error), 
			_last_time(0), _last_count(255), _lost_timeout(LOST_START_TIME),
			_error(ERROR_NONE), _error_last(ERROR_NONE)
		{}
		
		void SetOffset(int16_t offset)
		{
			_angle_offset = offset;
			
			return;
		}
		
		void SetInvert(bool invert)
		{
			_invert = invert;

			return;
		}
		
		bool PutPacket(uint32_t &time, uint16_t id, uint8_t *raw, uint8_t length)
		{
			_error = _CheckPacket(id, raw, length);
			if(_error != ERROR_NONE) return false;

			_lost_timeout = LOST_IDLE_TIME;
			
			return _Parse(time);
		}
		
		void Tick(uint32_t &time)
		{
			if(time - _last_time > _lost_timeout)
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
		
		error_t _CheckPacket(uint16_t id, uint8_t *raw, uint8_t length)
		{
			if(id != PACKET_ID) return ERROR_ID;
			if(length != PACKET_LENGTH) return ERROR_LENGTH;
			
			memcpy_reverse(_raw_array, raw, length);
			// Массив _raw_array кастится на объект _raw_obj, поэтому можно сразу работать с объектом

			if(_CalculateCRC() != _raw_obj->crc) return ERROR_CRC;
			if(_raw_obj->angle == 0xFFFF) return ERROR_ANGLE;
			if(_raw_obj->error != 0) return ERROR_ERR_SENSOR;
			if(_raw_obj->counter == _last_count) return ERROR_COUNTER;
			
			return ERROR_NONE;
		}

		bool _Parse(uint32_t &time)
		{
			_raw_obj->angle -= 0x8000;
			_raw_obj->roll -= 0x8000;
			
			_raw_obj->angle += _angle_offset;
			if(_invert == true)
			{
				_raw_obj->angle = (0 - _raw_obj->angle);
				_raw_obj->roll = (0 - _raw_obj->roll);
			}
			
			_sensor_data_float.angle = (float)_raw_obj->angle / 10.0f;
			_sensor_data_float.roll = (float)_raw_obj->roll / 10.0f;
			_sensor_data_float.dt = (float)(time - _last_time) / 1000.0f;
			
			_last_time = time;
			_last_count = _raw_obj->counter;

			return true;
		}
		
		uint8_t _CalculateCRC()
		{
			uint8_t crc = 0xFF;
			for(uint8_t i = 1; i < PACKET_LENGTH; ++i)
			{
				crc -= _raw_array[i];
			}
			return crc;
		}
		
		uint8_t _idx;
		callback_error_t _callback_error;
		
		uint32_t _last_time;
		uint8_t _last_count;
		
		uint8_t _raw_array[PACKET_LENGTH];
		sensor_packet_t *_raw_obj = (sensor_packet_t *) _raw_array;
		sensor_t _sensor_data_float;
		uint32_t _lost_timeout;

		error_t _error;
		error_t _error_last;

		int16_t _angle_offset = 0;
		bool _invert = false;
};
