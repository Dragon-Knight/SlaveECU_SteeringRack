#pragma once

extern TIM_HandleTypeDef htim4;


namespace SteeringRack
{
	
	static constexpr uint16_t PWM_MIN = 800;
	static constexpr uint16_t PWM_MID = 1500;
	static constexpr uint16_t PWM_MAX = 2200;

	static constexpr float PID_KP = 1.0f;
	static constexpr float PID_KI = 0.1f;
	static constexpr float PID_KD = 0.01f;


	struct params_t
	{
		
	};

	enum rack_id_t : uint8_t { RACK_1 = 0, RACK_2 = 1 };





class PIDController
{
	public:
		
		PIDController(float kp, float ki, float kd, float out_min, float out_max) : 
			_kp(kp), _ki(ki), _kd(kd), _out_min(out_min), _out_max(out_max), _integral(0), _previous_error(0)
		{}
		
		float Calculate(float setpoint, float measured_value, float dt)
		{
			float error = setpoint - measured_value;			// Вычисление ошибки
			float derivative = (error - _previous_error) / dt;	// Производная часть
			_integral += error * dt;							// Интегральная часть
			_previous_error = error;
			
			// Рассчитываем выход
			float output = _kp * error + _ki * _integral + _kd * derivative;
			
			// Ограничиваем выход для ШИМ
			//if(output > output_limit_max) output = output_limit_max;
			//else if(output < output_limit_min) output = output_limit_min;
			
			return output;
		}
		
	private:
		float _kp, _ki, _kd;		// Коэффициенты PID
		float _out_min, _out_max;	// Лимиты для расчёта PID
		float _integral;			// Накопление для интегральной части
		float _previous_error;		// Предыдущее значение ошибки


};





class SteeringControl
{
	public:
		SteeringControl(float p, float i, float d, uint16_t min_pwm, uint16_t mid_pwm, uint16_t max_pwm, TIM_HandleTypeDef *htim_pwm, uint32_t pwm_channel) : 
			_pid(p, i, d, min_pwm, max_pwm), _htim(htim_pwm), _channel(pwm_channel), _pwm_min(min_pwm), _pwm_mid(mid_pwm), _pwm_max(max_pwm), 
			_target(0.0f)
		{}

		void SetTarget(float target)
		{
			_target = target;

			return;
		}
		
		// Функция для обновления ПИД и управления ШИМ
		void Update(float measured_value, float dt)
		{
			float pid_output = _pid.Calculate(_target, measured_value, dt);
			uint16_t pwm = ((int32_t)(pid_output + 0.5f)) + (int32_t)_pwm_mid;
			uint16_t pwm_fix = clamp(pwm, _pwm_min, _pwm_max);
			
			DEBUG_LOG_TOPIC("Set pwm", "pid: %f, val: %d, val_fix: %d;\n", pid_output, pwm, pwm_fix);
			
			__HAL_TIM_SET_COMPARE(_htim, _channel, pwm_fix);
			
			return;
		}
		
	private:
		
		uint16_t clamp(uint16_t value, uint16_t min, uint16_t max)
		{
			return (value < min) ? min : (value > max) ? max : value;
		}
		
		PIDController _pid;			// Экземпляр PID-контроллера
		TIM_HandleTypeDef *_htim;	// Указатель на таймер для управления ШИМ
		uint32_t _channel;			// Канал ШИМ
		uint16_t _pwm_min, _pwm_mid, _pwm_max;
		float _target;
};

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
	
	void Tick(rack_id_t id, float angle, float roll, float dt)
	{
		steerings[id].Update(angle, dt);
		
		return;
	}
	
	
	inline void Setup()
	{
		HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);


		return;
	}
	
	inline void Loop(uint32_t &current_time)
	{
		
		
		current_time = HAL_GetTick();
		return;
	}
}
