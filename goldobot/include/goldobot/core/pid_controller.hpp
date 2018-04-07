#pragma once

namespace goldobot
{

	class FeedForwardPIDController
	{
	public:
		FeedForwardPIDController();

		float output() const;
		void set_target(float target, float target_derivative=0.0f);
		float do_step(float current_value, float dt);


	private:
		float m_kp;// Proportional coefficient
		float m_ki;// Integral coefficient
		float m_kd;// Derivative coefficient
		float m_ffp;// Proportional feedforward coefficient
		float m_ffd;// Derivative feedforward coefficient
		float m_lim_i;// Integral term limit
		float m_lim_d;// Derivative limit
		float m_current_target;
		float m_current_target_derivative;
		float m_previous_target;
	};
}
