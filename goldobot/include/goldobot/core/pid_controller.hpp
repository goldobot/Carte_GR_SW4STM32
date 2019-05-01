#pragma once

namespace goldobot
{
	struct PIDConfig
	{
		float period{1};
		float kp{0};
		float ki{0};
		float kd{0};
		float feed_forward{0};
		float lim_iterm{0};
		float lim_dterm{0};
		float min_output{0};
		float max_output{0};
	};

	class PIDController
	{
	public:
		PIDController();
		PIDController(const PIDConfig& config);

		const PIDConfig& config() const;
		void set_config(const PIDConfig& config);

		void set_kp(float kp);
		void set_ki(float ki);
		void set_kd(float kd);

		void set_feedforward(float feedforward);

		float output() const;
		void set_target(float target, float target_derivative=0.0f);
		void reset();
		float update(float current_value);


	private:
		float clamp(float val, float min_val, float max_val) const;

		PIDConfig m_config;

		float m_target{0};
		float m_target_derivative{0};
		float m_previous_value{0};
		float m_integral_term{0};
		float m_output{0};
		bool m_first_run{true};
	};
}
