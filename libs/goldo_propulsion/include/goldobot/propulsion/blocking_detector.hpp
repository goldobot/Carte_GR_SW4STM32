#pragma once
#include "goldobot/core/geometry.hpp"
#include "goldobot/core/low_pass_filter.hpp"

#include <cstdint>

namespace goldobot {
class PropulsionController;

namespace propulsion {

class BlockingDetector {
 public:
  enum Side {
      Left=0,
      Right=1
  };
  struct Config {
      float slip_speed_treshold;
      float shock_acceleration_treshold;
  };
      
  BlockingDetector();
  void setVelEstimates(float left, float right);
  void setTorqueEstimates(float left, float right);

  void update(const PropulsionController& controller);
  
  bool isSlip(Side side) const;  
  bool isBlocked() const;
  bool isShock() const;

  float m_speed_estimate;
  float m_force_estimate;

  float m_vel_estimates[2];
  float m_torque_estimates[2];

  LowPassFilter m_slip_speeds[2];
  int m_slip_counters[2];
};

}  // namespace propulsion
}  // namespace goldobot
