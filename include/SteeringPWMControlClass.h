#pragma once
#include <inttypes.h>
#include <math.h>
#include <CUtils.h>

class SteeringPWMControl
{
	static constexpr uint16_t PWM_OFF_VALUE = 0;
	static constexpr int16_t FEEDFORWARD_VALUE = 100;
	static constexpr int16_t DEAD_ZONE = 10;
	static constexpr float TARGET_MIN = -600.0;
	static constexpr float TARGET_MAX = 600.0;
	static constexpr float MEASURED_MAX = 610.0;
	
	public:
		
		SteeringPWMControl(float p, float i, float d, uint16_t min_pwm, uint16_t mid_pwm, uint16_t max_pwm, TIM_HandleTypeDef *htim_pwm, uint32_t pwm_channel) : 
			_pid(p, i, d, min_pwm, max_pwm), _htim(htim_pwm), _channel(pwm_channel), _pwm_min(min_pwm), _pwm_mid(mid_pwm), _pwm_max(max_pwm), 
			_target(0.0f), _isStopedPWM(true)
		{}

		void SetTarget(float target)
		{
			_target = clamp<float>(target, TARGET_MIN, TARGET_MAX);

			return;
		}
		
		void SetStopPWM()
		{
			_isStopedPWM = true;
			__HAL_TIM_SET_COMPARE(_htim, _channel, PWM_OFF_VALUE);
			_pid.Reset();
			
			return;
		}
		
		void SetStartPWM()
		{
			_isStopedPWM = false;
			_pid.Reset();
			
			return;
		}
		
		void Update(float measured_value, float dt)
		{
			if(_isStopedPWM == true) return;
			
			float pid_output = _pid.Calculate(_target, measured_value, dt);
			int32_t pwm = ((int32_t)(pid_output + 0.5f)) + (int32_t)_pwm_mid;
			uint16_t pwm_fix;
			
			// Задание начального тока, чтобы хватало мощности для поворота на маленьких значениях.
			if(pwm > _pwm_mid + DEAD_ZONE) pwm += FEEDFORWARD_VALUE;
			if(pwm < _pwm_mid - DEAD_ZONE) pwm -= FEEDFORWARD_VALUE;
			pwm_fix = clamp<int32_t>(pwm, _pwm_min, _pwm_max);
			
			DEBUG_LOG_TOPIC("Set pwm", "pid: %f, pwm: %d, pwm_fix: %d, target: %f, measured: %f;\n", pid_output, pwm, pwm_fix, _target, measured_value);
			
			if(abs(measured_value) > MEASURED_MAX)
			{
				SetStopPWM();
				
				DEBUG_LOG_TOPIC("pwm","STOP\n");
				
				return;
			}
			__HAL_TIM_SET_COMPARE(_htim, _channel, pwm_fix);
			
			return;
		}
		
	private:
		
		PIDController<float> _pid;	// Экземпляр PID-контроллера
		TIM_HandleTypeDef *_htim;	// Указатель на таймер для управления ШИМ
		uint32_t _channel;			// Канал ШИМ
		uint16_t _pwm_min, _pwm_mid, _pwm_max;
		float _target;
		bool _isStopedPWM;
};
