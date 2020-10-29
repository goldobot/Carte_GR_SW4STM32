#include "goldobot/core/trapezoidal_speed_profile.hpp"

#include <cmath>

using namespace goldobot;

namespace {
	constexpr float accelDistance(float speed_delta, float accel, float deccel)
	{
		return (speed_delta * speed_delta) * 0.5f / (speed_delta >= 0 ? accel : deccel);
	}

	constexpr float accelForDistance(float speed_delta, float distance)
	{
		return (speed_delta * speed_delta) * 0.5f / distance;
	}
}

TrapezoidalSpeedProfile::TrapezoidalSpeedProfile() {}

void TrapezoidalSpeedProfile::update(float distance, float speed, float accel, float deccel) {
  float d_a = (speed * speed) * 0.5f / accel;
  float d_d = (speed * speed) * 0.5f / deccel;
  float d_c = fabsf(distance) - d_a - d_d;

  while (d_c < 0) {
    speed *= 0.95;
    d_a = (speed * speed) * 0.5f / accel;
    d_d = (speed * speed) * 0.5f / deccel;
    d_c = fabsf(distance) - d_a - d_d;
  }

  float t_a = speed / accel;
  float t_d = speed / deccel;
  float t_c = d_c / speed;

  if (distance < 0) {
    d_a = -d_a;
    d_c = -d_c;
    d_d = -d_d;
    speed = -speed;
    accel = -accel;
    deccel = -deccel;
  }

  m_t[0] = 0;
  m_t[1] = t_a;
  m_t[2] = t_a + t_c;
  m_t[3] = t_a + t_c + t_d;

  m_c0[0] = 0;
  m_c0[1] = d_a;
  m_c0[2] = d_a + d_c;
  m_c0[3] = distance;

  m_c1[0] = 0;
  m_c1[1] = speed;
  m_c1[2] = speed;
  m_c1[3] = 0;

  m_c2[0] = 0.5 * accel;
  m_c2[1] = 0;
  m_c2[2] = -0.5 * deccel;
  m_c2[3] = 0;

  m_c3[0] = 0;
  m_c3[1] = 0;
  m_c3[2] = 0;
  m_c3[3] = 0;

  m_num_points = 4;
}

void TrapezoidalSpeedProfile::updateEx(float distance, float start_speed, float target_speed, float final_speed, float accel, float deccel)
{
	// First, check if we can reach final speed from start speed with given acceleration
	// If we cannot, compute needed acceleration to reach final speed from start_speed in distance
	if(distance < accelDistance(final_speed - start_speed, accel, deccel))
	{

	}

	// If target speed is between start and final speed
	// ramp to reach target speed, plateau, then ramp to final speed

	// If target speed is above or below both start and final speed
	// search speed limit

	// trajectory



}

float TrapezoidalSpeedProfile::begin_time() { return m_t[0]; }

float TrapezoidalSpeedProfile::end_time() { return m_t[3]; }

void TrapezoidalSpeedProfile::compute(float t, float* val, float* deriv, float* accel) {
  int index = 0;

  while (index + 1 < m_num_points && t > m_t[index + 1]) {
    index++;
  };

  float u = t - m_t[index];
  float c0 = m_c0[index];
  float c1 = m_c1[index];
  float c2 = m_c2[index];
  float c3 = m_c3[index];

  if (val != nullptr) {
    *val = c0 + u * (c1 + u * (c2 + u * c3));
  }
  if (deriv != nullptr) {
    *deriv = c1 + u * (2 * c2 + u * 3 * c3);
  }
  if (accel != nullptr) {
    *accel = 2 * c2 + 6 * u * c3;
  }
}
