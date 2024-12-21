#pragma once
#include <CUtils.h>

extern TIM_HandleTypeDef htim4;


namespace SteeringRack
{
	
	static constexpr uint16_t PWM_MIN = 800;
	static constexpr uint16_t PWM_MID = 1500;
	static constexpr uint16_t PWM_MAX = 2200;

	static constexpr float PID_KP = 1.0f;
	static constexpr float PID_KI = 0.1f;
	static constexpr float PID_KD = 0.01f;

	static constexpr float ANGLE_MID = 0.0f;



	enum rack_id_t : uint8_t
	{
		RACK_1 = 0, 		// Передняя рейка
		RACK_2 = 1,			// Задняя рейка
	};











	class SteeringControl
	{
		public:
			SteeringControl(float p, float i, float d, uint16_t min_pwm, uint16_t mid_pwm, uint16_t max_pwm, TIM_HandleTypeDef *htim_pwm, uint32_t pwm_channel) : 
				_pid(p, i, d, min_pwm, max_pwm), _htim(htim_pwm), _channel(pwm_channel), _pwm_min(min_pwm), _pwm_mid(mid_pwm), _pwm_max(max_pwm), 
				_target(0.0f), _isStopedPWM(true)
			{}

			void SetTarget(float target)
			{
				_target = target;

				return;
			}

			void SetStopPWM()
			{
				_isStopedPWM = true;
				__HAL_TIM_SET_COMPARE(_htim, _channel, 0);
			}

			void SetStartPWM()
			{
				_isStopedPWM = false;
			}
			
			// Функция для обновления ПИД и управления ШИМ
			void Update(float measured_value, float dt)
			{
				if(_isStopedPWM)
					return;

				float pid_output = _pid.Calculate(_target, measured_value, dt);
				uint16_t pwm = ((int32_t)(pid_output + 0.5f)) + (int32_t)_pwm_mid;
				uint16_t pwm_fix = clamp(pwm, _pwm_min, _pwm_max);
				
				//DEBUG_LOG_TOPIC("Set pwm", "pid: %f, val: %d, val_fix: %d;\n", pid_output, pwm, pwm_fix);
				__HAL_TIM_SET_COMPARE(_htim, _channel, pwm_fix);
				
				return;
			}
			
		private:
			
			uint16_t clamp(uint16_t value, uint16_t min, uint16_t max)
			{
				return (value < min) ? min : (value > max) ? max : value;
			}
			
			PIDController<float> _pid;	// Экземпляр PID-контроллера
			TIM_HandleTypeDef *_htim;	// Указатель на таймер для управления ШИМ
			uint32_t _channel;			// Канал ШИМ
			uint16_t _pwm_min, _pwm_mid, _pwm_max;
			float _target;
			bool _isStopedPWM;
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

	float prevAngleMasterOnModeChange = 0.0f;

	float angleMaster = 0.0f;
	float angleSlave = 0.0f;

	bool isSensorsCalculated = false;
	SteeringAngleSensorBase::error_t lastSensorErrorCode = SteeringAngleSensorBase::ERROR_NONE;








	SteeringControl steerings[] = 
	{
		{PID_KP, PID_KI, PID_KD, PWM_MIN, PWM_MID, PWM_MAX, &htim4, TIM_CHANNEL_1},
		{PID_KP, PID_KI, PID_KD, PWM_MIN, PWM_MID, PWM_MAX, &htim4, TIM_CHANNEL_2}
	};
	
	
	void SetTarget(rack_id_t id, float target)
	{
		steerings[id].SetTarget(target);
		
		return;
	}
	
	//получение актуального значения с датчиков
	void Tick(rack_id_t id, float angle, float roll, float dt)
	{
		if(id == RACK_1)
		{
			// Если была ошибка, то в CAN'е окажется послденее валидное значение.
			CANLib::obj_steering_angle.SetValue(0, (angle * 10), CAN_TIMER_TYPE_NORMAL);
			angleMaster = angle;
		}else
		{
			angleSlave = angle;
		}
		steerings[id].Update(angle, dt);
		isSensorsCalculated = true;
		
		return;
	}
	
	void OnErrorSensor (uint8_t id, SteeringAngleSensorBase::error_t code)
	{
		if (code > 0)
		{
			lastSensorErrorCode = code;
			OnChangeMode(STEERING_MODE_NONE);

		}
		else
		{
			if (lastSensorErrorCode == SteeringAngleSensorBase::ERROR_LOST)
			{
				OnChangeMode(STEERING_MODE_NONE);
			}
			else
			{
				OnChangeMode(mode);
			}
		}
	}

	void OnChangeMode(steering_mode_t mode)
	{
		prevAngleMasterOnModeChange = angleMaster;
		switch (mode)
		{
			case STEERING_MODE_NONE:
				//выключить шим
				steerings[RACK_1].SetStopPWM();
				steerings[RACK_2].SetStopPWM();
			break;
			case STEERING_MODE_STRAIGHT:
				steerings[RACK_1].SetStopPWM();
				steerings[RACK_2].SetStartPWM();
				steerings[RACK_2].SetTarget(0);
			break;
			case STEERING_MODE_REVERSE:
				steerings[RACK_1].SetStopPWM();
				steerings[RACK_2].SetStartPWM();
				steerings[RACK_2].SetTarget(ANGLE_MID - angleMaster);
			break;
			case STEERING_MODE_MIRROR:
				steerings[RACK_1].SetStopPWM();
				steerings[RACK_2].SetStartPWM();
				steerings[RACK_2].SetTarget(angleMaster);
			break;
			case STEERING_MODE_LOCK:

				steerings[RACK_1].SetStopPWM();
				steerings[RACK_2].SetStartPWM();
				steerings[RACK_2].SetTarget(prevAngleMasterOnModeChange);
			break;
			case STEERING_MODE_REMOTE:
			//пока не управляем
				steerings[RACK_1].SetStopPWM();
				steerings[RACK_2].SetStopPWM();
			break;
			default:
				steerings[RACK_1].SetStopPWM();
				steerings[RACK_2].SetStopPWM();
			break;
		}		
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
			
			can_frame.function_id = CAN_FUNC_EVENT_OK;
			return CAN_RESULT_CAN_FRAME;
		});


		return;
	}
	
	inline void Loop(uint32_t &current_time)
	{
		if (isSensorsCalculated)
		{
			switch (mode)
			{
				case STEERING_MODE_NONE:

				break;
				case STEERING_MODE_STRAIGHT:

				break;
				case STEERING_MODE_REVERSE:
					steerings[RACK_2].SetTarget(ANGLE_MID - angleMaster);
				break;
				case STEERING_MODE_MIRROR:
					steerings[RACK_2].SetTarget(angleMaster);
				break;
				case STEERING_MODE_LOCK:
				
				break;
				case STEERING_MODE_REMOTE:

				break;
				default:
					steerings[RACK_1].SetStopPWM();
					steerings[RACK_2].SetStopPWM();				
				break;
			}
			isSensorsCalculated = false;
		}

		current_time = HAL_GetTick();
		return;
	}
}
