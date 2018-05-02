#pragma once
#include "goldobot/core/trapezoidal_speed_profile.hpp"

namespace goldobot
{
	class TrapezoidalSpeedProfile
	{
	public:
		TrapezoidalSpeedProfile();
		void update(float distance, float speed, float accel, float deccel);
		float begin_time();
		float end_time();
		void compute(float t, float* val, float* deriv, float* accel);

	private:
		float m_c0[4];
		float m_c1[4];
		float m_c2[4];
		float m_t[4];
	};
}
