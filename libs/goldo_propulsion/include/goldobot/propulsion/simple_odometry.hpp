#pragma once
#include "goldobot/core/derivative_filter.hpp"
#include "goldobot/core/geometry.hpp"
#include "goldobot/propulsion/odometry_config.hpp"

#include <cstdint>

namespace goldobot {
class SimpleOdometry {
 public:
  SimpleOdometry() = default;

  uint16_t leftEncoderValue() const;
  uint16_t rightEncoderValue() const;

  uint32_t leftAccumulator() const;
  uint32_t rightAccumulator() const;

  void setPeriod(float period);

  const OdometryConfig& config() const;
  void setConfig(const OdometryConfig& config);

  const RobotPose& pose() const;
  void setPose(const RobotPose& pose);

  //! \brief Update current pose to be on specified line, with yaw normal to line.
  //! Used for repositioning on table borders
  void measureLineNormal(Vector2D normal, float distance);
  void measurePerpendicularPoint(float angle, float offset, Vector2D point);
  void reset(uint16_t left_encoder, uint16_t right_encoder);
  void update(uint16_t left_encoder, uint16_t right_encoder);

  float m_diff_left{0};
 private:
  uint16_t m_left_encoder{0};
  uint16_t m_right_encoder{0};
  int32_t m_left_accumulator{0};
  int32_t m_right_accumulator{0};
  RobotPose m_pose;
  float m_period{0};
  uint16_t m_encoders_period{8192};
  OdometryConfig m_config;

  double m_x{0};
  double m_y{0};
  float m_yaw{0};
  DerivativeFilter m_speed_filter;
  DerivativeFilter m_yaw_rate_filter;
  DerivativeFilter m_accel_filter;
  DerivativeFilter m_angular_accel_filter;

  void updatePose(float dx, float dtheta, float dt);
};
}  // namespace goldobot
