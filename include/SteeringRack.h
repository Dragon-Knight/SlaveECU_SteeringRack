#pragma once
#include "SteeringPWMControlClass.h"
#include "SteeringAngleSensorBase.h"

extern TIM_HandleTypeDef htim4;


namespace SteeringRack
{
	
	static constexpr uint16_t PWM_MIN = 800;
	static constexpr uint16_t PWM_MID = 1500;
	static constexpr uint16_t PWM_MAX = 2200;

	static constexpr float PID_KP = 3.0f;
	static constexpr float PID_KI = 0.1f;
	static constexpr float PID_KD = 0.01f;

	static constexpr float ANGLE_MID = 0.0f;

	

	enum rack_id_t : uint8_t
	{
		RACK_1 = 0, 		// Передняя рейка
		RACK_2 = 1,			// Задняя рейка
	};

	enum steering_mode_t : uint8_t
	{
		STEERING_MODE_NONE = 0,			// Полностью отключенное электронное управление поворотами
		STEERING_MODE_STRAIGHT = 1,		// Задняя ось выравнивается в нулевое положение. Положение передней оси игнорируется
		STEERING_MODE_REVERSE = 2,		// Задняя ось выравнивается на обратный угол относительно передней оси
		STEERING_MODE_MIRROR = 3,		// Задняя ось выравнивается на тот же угол равный передней оси
		STEERING_MODE_LOCK = 4,			// Задняя ось в режиме фиксации выставленного угла (устанавливается последнее фактическое значение задней оси)
		STEERING_MODE_REMOTE = 128		// Режим удалённого управления
	};
	
	steering_mode_t mode = STEERING_MODE_NONE;
	float target = 0.0f;

	//float prevAngleMasterOnModeChange = 0.0f;

	float angleMaster = 0.0f;
	float angleSlave = 0.0f;

	bool isSensorsCalculated = false;
	SteeringAngleSensorBase::error_t lastSensorErrorCode = SteeringAngleSensorBase::ERROR_NONE;

	SteeringPWMControl steerings[] = 
	{
		{PID_KP, PID_KI, PID_KD, PWM_MIN, PWM_MID, PWM_MAX, &htim4, TIM_CHANNEL_1},
		{PID_KP, PID_KI, PID_KD, PWM_MIN, PWM_MID, PWM_MAX, &htim4, TIM_CHANNEL_2}
	};
	


	void OnChangeMode(steering_mode_t mode);



	// Получение актуального значения с датчиков
	void OnDataSensor(rack_id_t id, float angle, float roll, float dt)
	{
		if(id == RACK_1)
		{
			// Если была ошибка, то в CAN'е окажется послденее валидное значение.
			CANLib::obj_steering_angle_front.SetValue(0, (angle * 10), CAN_TIMER_TYPE_NORMAL);
			
			angleMaster = angle;
		} else {
			// Если была ошибка, то в CAN'е окажется послденее валидное значение.
			CANLib::obj_steering_angle_rear.SetValue(0, (angle * 10), CAN_TIMER_TYPE_NORMAL);

			angleSlave = angle;
		}
		steerings[id].Update(angle, dt);
		isSensorsCalculated = true;

		//DEBUG_LOG_TOPIC("RACK", "id: %d, a: %f\n",id ,angle);

		
		return;
	}
	
	// Получение кода ошибки с датчиков
	void OnErrorSensor(rack_id_t id, SteeringAngleSensorBase::error_t code)
	{
		if(code > 0)
		{
			lastSensorErrorCode = code;
			OnChangeMode(STEERING_MODE_NONE);

		}
		else
		{
			if(lastSensorErrorCode == SteeringAngleSensorBase::ERROR_LOST)
			{
				OnChangeMode(STEERING_MODE_NONE);
			}
			else
			{
				OnChangeMode(mode);
			}
		}
	}
	
	// Получение по CANу команды установки режима
	void OnChangeMode(steering_mode_t mode)
	{
		//prevAngleMasterOnModeChange = angleMaster;
		
		switch(mode)
		{
			case STEERING_MODE_NONE:
			{
				//выключить шим
				steerings[RACK_1].SetStopPWM();
				steerings[RACK_2].SetStopPWM();
				break;
			}
			case STEERING_MODE_STRAIGHT:
			{
				steerings[RACK_1].SetStopPWM();
				steerings[RACK_2].SetStartPWM();
				steerings[RACK_2].SetTarget(ANGLE_MID);
				break;
			}
			case STEERING_MODE_REVERSE:
			{
				steerings[RACK_1].SetStopPWM();
				steerings[RACK_2].SetStartPWM();
				steerings[RACK_2].SetTarget(ANGLE_MID - angleMaster);
				break;
			}
			case STEERING_MODE_MIRROR:
			{
				steerings[RACK_1].SetStopPWM();
				steerings[RACK_2].SetStartPWM();
				steerings[RACK_2].SetTarget(angleMaster);
				break;
			}
			case STEERING_MODE_LOCK:
			{
				steerings[RACK_1].SetStopPWM();
				steerings[RACK_2].SetStartPWM();
				steerings[RACK_2].SetTarget(angleMaster);
				break;
			}
			case STEERING_MODE_REMOTE:
			{
				//пока не управляем
				steerings[RACK_1].SetStopPWM();
				steerings[RACK_2].SetStopPWM();
				break;
			}
			default:
			{
				steerings[RACK_1].SetStopPWM();
				steerings[RACK_2].SetStopPWM();
				break;
			}
		}
		
		return;
	}
	
	// Получение по CANу команды установки угла поворота
	void OnChangeTarget(float target)
	{


		return;
	}
	
	
	inline void Setup()
	{
		HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);

		CANLib::obj_turn_mode.RegisterFunctionSet([](can_frame_t &can_frame, can_error_t &error) -> can_result_t
		{
			mode = (steering_mode_t)can_frame.data[0];
			
			OnChangeMode(mode);
			
			can_frame.function_id = CAN_FUNC_EVENT_OK;
			return CAN_RESULT_CAN_FRAME;
		});
		
		CANLib::obj_target_angle.RegisterFunctionSet([](can_frame_t &can_frame, can_error_t &error) -> can_result_t
		{
			target = (float)(can_frame.data[0] | (can_frame.data[1] << 8)) / 10.0f;

			OnChangeTarget(target);
			
			can_frame.function_id = CAN_FUNC_EVENT_OK;
			return CAN_RESULT_CAN_FRAME;
		});


		return;
	}
	
	inline void Loop(uint32_t &current_time)
	{
		if(isSensorsCalculated == true)
		{
			isSensorsCalculated = false;
			
			switch (mode)
			{
				case STEERING_MODE_NONE:
				{
					break;
				}
				case STEERING_MODE_STRAIGHT:
				{
					break;
				}
				case STEERING_MODE_REVERSE:
				{
					steerings[RACK_2].SetTarget(ANGLE_MID - angleMaster);
					break;
				}
				case STEERING_MODE_MIRROR:
				{
					steerings[RACK_2].SetTarget(angleMaster);
					break;
				}
				case STEERING_MODE_LOCK:
				{
					break;
				}
				case STEERING_MODE_REMOTE:
				{
					break;
				}
				default:
				{
					steerings[RACK_1].SetStopPWM();
					steerings[RACK_2].SetStopPWM();				
					break;
				}
			}
		}
		
		current_time = HAL_GetTick();
		
		return;
	}
}
