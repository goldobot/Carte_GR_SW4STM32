#pragma once
#include "goldobot/core/geometry.hpp"

#include <cstdint>

namespace goldobot {
class PropulsionController;

namespace propulsion {

class BlockingDetector {
public:

	void setVelEstimates(float left, float right);
	void setTorqueEstimates(float left, float right);
	
	
	void update(const PropulsionController& controller);
	
	float m_speed_estimate;

	
	float m_vel_estimates[2];
	float m_torque_estimates[2];
};




}
}

