#pragma once

namespace goldobot
{
	struct PIDConfig
	{
		PIDConfig();
		float period;
		float kp;
		float kd;
		float ki;
		float feed_forward;
		float lim_iterm;
		float lim_dterm;
		float min_output;
		float max_output;
	};

	class PIDController
	{
	public:
		PIDController();
		PIDController(const PIDConfig& config);

		const PIDConfig& config() const;
		void set_config(const PIDConfig& config);


		void set_kp(float kp);
		void set_kd(float kd);
		void set_ki(float ki);
		void set_feedforward(float feedforward);

		float output() const;
		void set_target(float target, float target_derivative=0.0f);
		void reset();
		float update(float current_value);


	private:
		float clamp(float val, float min_val, float max_val) const;

		PIDConfig m_config;

		float m_target;
		float m_target_derivative;
		float m_previous_value;
		float m_integral_term;
		float m_output;
		bool m_first_run;
	};
}
