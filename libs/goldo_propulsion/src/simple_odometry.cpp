#include "goldobot/core/math_utils.hpp"

#include <goldobot/propulsion/simple_odometry.hpp>

#define _USE_MATH_DEFINES
#include <cmath>
#include <cstring>

using namespace goldobot;

uint16_t SimpleOdometry::leftEncoderValue() const { return m_left_encoder; }

uint16_t SimpleOdometry::rightEncoderValue() const { return m_right_encoder; }

uint32_t SimpleOdometry::leftAccumulator() const { return m_left_accumulator; }
uint32_t SimpleOdometry::rightAccumulator() const { return m_right_accumulator; }

void SimpleOdometry::setPeriod(float period) { m_period = period; }

const RobotPose& SimpleOdometry::pose() const { return m_pose; }

const OdometryConfig& SimpleOdometry::config() const { return m_config; }

void SimpleOdometry::setConfig(const OdometryConfig& config) {
  m_config = config;
  m_speed_filter.setConfig(m_period, config.speed_filter_frequency);
  m_accel_filter.setConfig(m_period, config.accel_filter_frequency);
  m_yaw_rate_filter.setConfig(m_period, config.speed_filter_frequency);
  m_angular_accel_filter.setConfig(m_period, config.accel_filter_frequency);
}

void SimpleOdometry::reset(uint16_t left, uint16_t right) {
  m_left_encoder = left;
  m_right_encoder = right;
  std::memset(&m_pose, 0, sizeof(m_pose));
}

void SimpleOdometry::update(uint16_t left, uint16_t right) {
  int diff_left = left - m_left_encoder;
  int diff_right = right - m_right_encoder;

  m_left_encoder = left;
  m_right_encoder = right;

  if (diff_left * 2 > m_encoders_period) {
    diff_left -= m_encoders_period;
  }
  if (diff_left * 2 < -m_encoders_period) {
    diff_left += m_encoders_period;
  }
  if (diff_right * 2 > m_encoders_period) {
    diff_right -= m_encoders_period;
  }
  if (diff_right * 2 < -m_encoders_period) {
    diff_right += m_encoders_period;
  }

  m_left_accumulator += diff_left;
  m_right_accumulator += diff_right;

  double d_left = diff_left * m_config.dist_per_count_left;
  double d_right = diff_right * m_config.dist_per_count_right;
  double d_yaw =
      (d_right - d_left) / (m_config.wheel_distance_left + m_config.wheel_distance_right);
  double d_trans = (d_left + d_right) * 0.5;

  m_x += d_trans * cos(m_yaw + d_yaw * 0.5);
  m_y += d_trans * sin(m_yaw + d_yaw * 0.5);
  m_yaw = clampAngle(m_yaw + d_yaw);

  float previous_speed = m_pose.speed;
  float previous_yaw_rate = m_pose.yaw_rate;

  m_pose.position.x = static_cast<float>(m_x);
  m_pose.position.y = static_cast<float>(m_y);
  m_pose.yaw = static_cast<float>(m_yaw);

  // Speed filter
  m_pose.speed = m_speed_filter.step(d_trans);
  m_pose.yaw_rate = m_yaw_rate_filter.step(d_yaw);

  // Acceleration filter
  float d_speed = m_pose.speed - previous_speed;
  float d_yaw_rate = previous_yaw_rate - m_pose.yaw_rate;
  m_pose.acceleration = m_accel_filter.step(d_speed);
  m_pose.angular_acceleration = m_angular_accel_filter.step(d_yaw_rate);
}

void SimpleOdometry::setPose(const RobotPose& pose) {
  m_pose = pose;
  m_x = pose.position.x;
  m_y = pose.position.y;
  m_yaw = pose.yaw;
  m_speed_filter.reset(m_pose.speed);
  m_yaw_rate_filter.reset(m_pose.yaw_rate);
  m_accel_filter.reset(m_pose.acceleration);
  m_angular_accel_filter.reset(m_pose.angular_acceleration);
}

void SimpleOdometry::measureLineNormal(Vector2D normal, float distance) {
  // Project position on line
  double new_x = normal.y * normal.y * m_x - normal.x * normal.y * m_y + normal.x * distance;
  double new_y = -normal.x * normal.y * m_x + normal.x * normal.x * m_y + normal.y * distance;

  // Select sign of new yaw to be consistent with current
  float dp = normal.x * cos(m_yaw) + normal.y * sin(m_yaw);
  if (dp >= 0) {
    m_yaw = atan2(normal.y, normal.x);
  } else {
    m_yaw = atan2(-normal.y, -normal.x);
  }

  m_x = new_x;
  m_y = new_y;

  m_pose.position.x = static_cast<float>(m_x);
  m_pose.position.y = static_cast<float>(m_y);
  m_pose.yaw = static_cast<float>(m_yaw);
}

void SimpleOdometry::measurePerpendicularPoint(float angle, float offset, Vector2D point) {
  // update longitudinal position so that the line having angle angle from robot x axis and
  // intersection offset with robot x axis is coincident with point

  double ux = cos(m_yaw);
  double uy = sin(m_yaw);

  double pt_xrel = (point.x - m_x) * ux + (point.y - m_y) * uy;

  // todo : now only look perpendicular to robot axis
  // double pt_yrel = point.x * uy - point.y * ux;

  m_x += ux * (pt_xrel - offset);
  m_y += uy * (pt_xrel - offset);
  m_pose.position.x = static_cast<float>(m_x);
  m_pose.position.y = static_cast<float>(m_y);
}
