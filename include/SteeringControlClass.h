#pragma once
#include <inttypes.h>
#include <CUtils.h>

class SteeringPWMControl
{
	static constexpr uint16_t PWM_OFF_VALUE = 0;
	
	public:
		
		SteeringPWMControl(float p, float i, float d, uint16_t min_pwm, uint16_t mid_pwm, uint16_t max_pwm, TIM_HandleTypeDef *htim_pwm, uint32_t pwm_channel) : 
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
			DEBUG_LOG_TOPIC("SetStopPWM", "SetStopPWM-in : %d\n", _channel);
			_isStopedPWM = true;
			__HAL_TIM_SET_COMPARE(_htim, _channel, PWM_OFF_VALUE);
			
			return;
		}
		
		void SetStartPWM()
		{
			DEBUG_LOG_TOPIC("SetStartPWM", "SetStartPWM-in : %d\n", _channel);
			_isStopedPWM = false;
			
			return;
		}
		
		void Update(float measured_value, float dt)
		{
			DEBUG_LOG_TOPIC("Update", "Update-in : %d\n", _channel);
			if(_isStopedPWM == true) return;
			
			float pid_output = _pid.Calculate(_target, measured_value, dt);
			uint16_t pwm = ((int32_t)(pid_output + 0.5f)) + (int32_t)_pwm_mid;
			uint16_t pwm_fix = clamp<uint16_t>(pwm, _pwm_min, _pwm_max);
			
			DEBUG_LOG_TOPIC("Set pwm", "pid: %f, val: %d, val_fix: %d;\n", pid_output, pwm, pwm_fix);
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
