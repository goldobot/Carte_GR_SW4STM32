#include "goldobot/propulsion/controller.hpp"
#include "goldobot/propulsion/simple_odometry.hpp"
#include "goldobot/core/math_utils.hpp"

using namespace goldobot;

#include <algorithm>

PropulsionController::PropulsionController(SimpleOdometry* odometry) : m_odometry(odometry) {}

void PropulsionController::setEnable(bool enable) {
  if (enable && m_state == State::Inactive) {
    m_current_pose = m_odometry->pose();
    m_target_pose = m_current_pose;
    m_low_level_controller.reset();
    setState(State::Stopped);
  }
  if (!enable && m_state != State::Inactive) {
    m_low_level_controller.reset();
    setState(State::Inactive, Error::None);
  }
}

PropulsionController::State PropulsionController::state() const { return m_state; }

PropulsionController::Error PropulsionController::error() const { return m_error; }

bool PropulsionController::stateChanged() {
  auto state_changed = m_state_changed;
  m_state_changed = false;
  return state_changed;
}

const PropulsionControllerConfig& PropulsionController::config() const { return m_config; }

void PropulsionController::setConfig(const PropulsionControllerConfig& config) {
  m_config = config;
  m_low_level_controller.setConfig(config.low_level_config);
  m_low_level_controller.setPidConfig(m_config.pid_configs[0]);
  m_low_level_controller.reset();
}

void PropulsionController::clearError() {
  if (m_state != State::Error) {
    return;
  }

  setState(State::Stopped, Error::None);

  m_target_pose = m_current_pose;
  m_low_level_controller.reset();
}

bool PropulsionController::commandFinished() { return m_command_finished; }

void PropulsionController::emergencyStop() {
  switch (m_state) {
    case State::FollowTrajectory:
      m_speed_controller.setAccelerationLimits(2, 2);
      m_speed_controller.setRequestedSpeed(0);
      m_emergency_stop = true;
      return;
    case State::Rotate:
      m_speed_controller.setAccelerationLimits(2, 2);
      m_speed_controller.setRequestedSpeed(0);
      m_emergency_stop = true;
      return;
    default:
      return;
  }
}

void PropulsionController::setAccelerationLimits(float accel, float deccel, float angular_accel,
                                                 float angular_deccel) {
  m_accel = accel;
  m_deccel = deccel;
  m_angular_accel = angular_accel;
  m_angular_deccel = angular_deccel;
}

void PropulsionController::setTargetSpeed(float speed) {
  if (m_emergency_stop) {
    return;
  }
  switch (m_state) {
    case State::FollowTrajectory:
      m_speed_controller.setRequestedSpeed(speed);
      return;
    case State::Rotate:
      m_speed_controller.setRequestedSpeed(speed);
      return;
    default:
      return;
  }
}

void PropulsionController::update() {
  m_command_finished = false;
  m_current_pose = m_odometry->pose();
  switch (m_state) {
    case State::Inactive:
      break;
    case State::Stopped:
      if (fabsf(m_low_level_controller.m_longi_error) > 0.1f) {
        setState(State::Error, Error::TrackingError);
      }
      break;
    case State::FollowTrajectory: {
      m_speed_controller.update();
      updateTargetPositions();
      check_tracking_error();
      if (m_speed_controller.finished()) {
        if (m_reposition_distance != 0) {
          setState(State::Reposition);
        } else {
          on_command_finished();
        }
      }
      if (m_emergency_stop && fabsf(m_speed_controller.speed()) < 1e-3f) {
        on_command_finished();
      }
    } break;
    case State::Rotate: {
      m_speed_controller.update();
      updateTargetYaw();
      check_tracking_error();
      if (m_speed_controller.finished()) {
        on_command_finished();
      }
      if (m_emergency_stop && fabsf(m_speed_controller.speed()) < 1e-3f) {
        on_command_finished();
      }
    } break;
    case State::Reposition: {
      m_speed_controller.update();
      updateReposition();
    } break;
    case State::ManualControl:
      break;
    case State::Error:
      m_low_level_controller.reset();
      break;
    default:
      break;
  }

  if (m_state != State::Inactive && m_state != State::Error && m_state != State::EmergencyStop) {
    updateMotorsPwm();
  }
  m_blocking_detector.update(*this);
  // Update time base
  m_time_base_ms++;
}

float PropulsionController::leftMotorVelocityInput() const noexcept {
  return m_low_level_controller.m_left_motor_velocity_input;
}

float PropulsionController::PropulsionController::rightMotorVelocityInput() const noexcept {
  return m_low_level_controller.m_right_motor_velocity_input;
}

float PropulsionController::leftMotorTorqueInput() const noexcept {
  return m_low_level_controller.m_left_motor_torque_input;
}

float PropulsionController::rightMotorTorqueInput() const noexcept {
  return m_low_level_controller.m_right_motor_torque_input;
}

float PropulsionController::leftMotorTorqueLimit() const noexcept {
  return m_low_level_controller.m_left_motor_torque_lim;
}

float PropulsionController::rightMotorTorqueLimit() const noexcept {
  return m_low_level_controller.m_right_motor_torque_lim;
}

void PropulsionController::setMotorsVelEstimates(float left, float right) {
  m_blocking_detector.setVelEstimates(left, right);
}

void PropulsionController::setMotorsTorqueEstimates(float left, float right) {
  m_blocking_detector.setTorqueEstimates(left, right);
}

const RobotPose& PropulsionController::targetPose() const { return m_target_pose; }

const RobotPose& PropulsionController::currentPose() const { return m_current_pose; }

void PropulsionController::setTargetPose(const RobotPose& target_pose) {
  m_target_pose = target_pose;
}

void PropulsionController::setControlLevels(uint8_t longi, uint8_t yaw) {
  m_low_level_controller.m_longi_control_level = longi;
  m_low_level_controller.m_yaw_control_level = yaw;
}

void PropulsionController::updateTargetPositions() {
  // Compute current distance on trajectory target
  // float t = (m_time_base_ms - m_command_begin_time) * 1e-3f;

  float parameter = m_speed_controller.parameter();
  float speed = m_speed_controller.speed();
  parameter = std::min(parameter, m_trajectory_buffer.max_parameter());

  // Compute position of lookahead point in front of current position
  float lookahead_distance = m_config.lookahead_distance + fabsf(speed) * m_config.lookahead_time;
  float lookahead_parameter = parameter + lookahead_distance;

  // Compute target position
  auto target_point = m_trajectory_buffer.compute_point(parameter);
  m_target_pose.position = target_point.position;

  // Compute position of lookahead point
  // Extend the trajectory past last point if necessary
  auto max_parameter = m_trajectory_buffer.max_parameter();
  if (lookahead_parameter <= max_parameter) {
    m_lookahead_position = m_trajectory_buffer.compute_point(lookahead_parameter).position;
  } else {
    auto end_point = m_trajectory_buffer.compute_point(max_parameter);
    m_lookahead_position.x =
        end_point.position.x + (lookahead_parameter - max_parameter) * end_point.tangent.x;
    m_lookahead_position.y =
        end_point.position.y + (lookahead_parameter - max_parameter) * end_point.tangent.y;
  }

  // Compute speed. project trajectory speed on robot axis, to get correct value during curves
  // This also has the consequence of slowing in curves
  // Current robot frame direction
  float ux = cosf(m_current_pose.yaw);
  float uy = sinf(m_current_pose.yaw);

  m_target_pose.speed = speed * (ux * target_point.tangent.x + uy * target_point.tangent.y);

  // Pure pursuit computation, update target yaw rate
  float diff_x = m_lookahead_position.x - m_current_pose.position.x;
  float diff_y = m_lookahead_position.y - m_current_pose.position.y;

  float rel_x = diff_x * ux + diff_y * uy;
  float rel_y = -diff_x * uy + diff_y * ux;

  float curvature = 2 * rel_y / (rel_x * rel_x + rel_y * rel_y);
  m_target_pose.yaw_rate = m_current_pose.speed * curvature;
  m_target_pose.yaw = m_current_pose.yaw;
}

void PropulsionController::updateTargetYaw() {
  float parameter = m_speed_controller.parameter();
  float speed = m_speed_controller.speed();
  m_target_pose.yaw = clampAngle(m_begin_yaw + parameter * m_rotation_direction);
  m_target_pose.yaw_rate = speed * m_rotation_direction;
}

void PropulsionController::updateMotorsPwm() {
  // Execute low level control
  m_low_level_controller.update(m_current_pose, m_target_pose);
}

void PropulsionController::updateReposition() {
  float parameter = m_speed_controller.parameter();
  float speed = m_speed_controller.speed();
  parameter = std::min(parameter, m_trajectory_buffer.max_parameter());

  // Compute target position
  float sign = m_direction == Direction::Forward ? 1.0f : -1.0f;
  auto target_point = m_trajectory_buffer.compute_point(parameter);
  m_target_pose.position = target_point.position;
  m_target_pose.speed = speed * sign;
  m_target_pose.yaw = atan2f(target_point.tangent.y, target_point.tangent.x) +
                      (m_direction == Direction::Forward ? 0 : c_pi);
  m_target_pose.yaw_rate = 0;

  // detect slipping
  float slip_speed_treshold = 0.15f;
  if (fabsf(m_blocking_detector.m_slip_speeds[0].value()) > slip_speed_treshold) {
    m_low_level_controller.m_left_motor_torque_lim = m_config.reposition_torque_limit;
  }
  if (fabsf(m_blocking_detector.m_slip_speeds[1].value()) > slip_speed_treshold) {
    m_low_level_controller.m_right_motor_torque_lim = m_config.reposition_torque_limit;
  }
  if (fabs(m_low_level_controller.m_longi_error) > 0.03 && !m_reposition_hit) {
    m_reposition_hit = true;
    m_reposition_end_ts = m_time_base_ms + 200;
  }

  if (m_speed_controller.finished() && !m_reposition_hit) {
    setState(State::Stopped);
  }

  if (m_reposition_hit && m_time_base_ms >= m_reposition_end_ts) {
    setState(State::Stopped);
  }
};

void PropulsionController::check_tracking_error() {
  bool error = false;
  if (m_state == State::FollowTrajectory || m_state == State::Rotate) {
    if (fabsf(m_low_level_controller.m_lateral_error) >= 0.2f) {
      error = true;
    }
    if (fabsf(m_low_level_controller.m_longi_error) >= 0.3f) {
      error = true;
    }
    if (fabsf(m_low_level_controller.m_yaw_error) >= 0.5f) {
      error = true;
    }
    if (error) {
      m_command_finished = true;
    }
  }
  if (error == true) {
    setState(State::Error, Error::TrackingError);
  }
}

void PropulsionController::on_stopped_enter() {
  m_low_level_controller.setPidConfig(m_config.pid_configs[0]);
  m_low_level_controller.m_longi_control_level = 2;
  m_low_level_controller.m_yaw_control_level = 2;

  m_low_level_controller.m_left_motor_torque_lim = m_config.static_torque_limit;
  m_low_level_controller.m_right_motor_torque_lim = m_config.static_torque_limit;

  m_target_pose.speed = 0;
  m_target_pose.yaw_rate = 0;
  m_target_pose.acceleration = 0;
  m_target_pose.angular_acceleration = 0;

  m_low_level_controller.m_motor_velocity_limit = m_config.static_pwm_limit;
  m_command_finished = true;
  m_emergency_stop = false;
}

void PropulsionController::setState(State state, Error error) {
  m_error = error;
  setState(state);
  m_state_changed = true;
}
void PropulsionController::setState(State state) {
  if (state == m_state) {
    return;
  }

  switch (m_state) {
    case State::Reposition:
      onRepositionExit();
      break;
    default:
      break;
  }

  switch (state) {
    case State::Stopped:
      on_stopped_enter();
      break;
    case State::FollowTrajectory:
      onFollowTrajectoryEnter();
      break;
    case State::Reposition:
      onRepositionEnter();
      break;
    default:
      break;
  };
  m_state_changed = true;
  m_state = state;
}
void PropulsionController::on_command_finished() {
  if (m_emergency_stop) {
    setState(State::Error, Error::EmergencyStop);
    m_command_finished = true;
    m_low_level_controller.reset();
  } else {
    setState(State::Stopped);
  }
}

void PropulsionController::onFollowTrajectoryEnter() {
  m_low_level_controller.m_motor_velocity_limit = m_config.cruise_pwm_limit;
  m_low_level_controller.m_left_motor_torque_lim = m_config.cruise_torque_limit;
  m_low_level_controller.m_right_motor_torque_lim = m_config.cruise_torque_limit;

  m_low_level_controller.setPidConfig(m_config.pid_configs[0]);
  m_low_level_controller.m_longi_control_level = 2;
  m_low_level_controller.m_yaw_control_level = 1;
}

void PropulsionController::onRepositionEnter() {
  // compute target point
  Vector2D target;

  float ux = cos(m_target_pose.yaw);
  float uy = sin(m_target_pose.yaw);

  target.x = m_target_pose.position.x + ux * m_reposition_distance;
  target.y = m_target_pose.position.y + uy * m_reposition_distance;

  Vector2D traj[2];
  traj[0] = m_target_pose.position;
  traj[1] = target;

  m_trajectory_buffer.push_segment(traj, 2);

  m_speed_controller.setAccelerationLimits(m_accel, m_deccel);
  m_speed_controller.setParameterRange(0, m_trajectory_buffer.max_parameter());
  m_speed_controller.setRequestedSpeed(m_reposition_speed);
  m_speed_controller.setFinalSpeed(0);
  m_speed_controller.reset(0, m_target_pose.speed, m_target_pose.acceleration);

  m_reposition_hit = false;
  m_low_level_controller.setPidConfig(m_config.pid_configs[0]);
  m_low_level_controller.m_longi_control_level = 2;
  m_low_level_controller.m_yaw_control_level = 2;
}

void PropulsionController::onRepositionExit() {
  auto pose = m_odometry->pose();
  m_target_pose.position = pose.position;
  m_target_pose.yaw = pose.yaw;
  m_target_pose.speed = 0;
  m_target_pose.yaw_rate = 0;
  m_reposition_speed = 0;
  m_reposition_distance = 0;
}

/*
void PropulsionController::initMoveCommand(float speed) {
  m_speed_controller.setAccelerationLimits(m_accel, m_deccel);
  m_speed_controller.setParameterRange(0, m_trajectory_buffer.max_parameter());
  m_speed_controller.setRequestedSpeed(speed);
  m_speed_controller.reset(0, m_target_pose.speed, m_target_pose.acceleration);

  // Compute direction by taking scalar product of current robot orientation vector with tangent of
  // trajectory at origin
  auto target_point = m_trajectory_buffer.compute_point(0);
  float ux = cosf(m_target_pose.yaw);
  float uy = sinf(m_target_pose.yaw);
  m_direction = ux * target_point.tangent.x + uy * target_point.tangent.y > 0 ? Direction::Forward
                                                                              : Direction::Backward;
  m_low_level_controller.m_motor_velocity_limit = m_config.cruise_pwm_limit;
  m_low_level_controller.m_left_motor_torque_lim = m_config.cruise_torque_limit;
  m_low_level_controller.m_right_motor_torque_lim = m_config.cruise_torque_limit;
}*/

bool PropulsionController::resetPose(float x, float y, float yaw) {
  RobotPose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.yaw = yaw;
  pose.speed = 0;
  pose.yaw_rate = 0;
  m_odometry->setPose(pose);
  m_target_pose = pose;
  m_low_level_controller.reset();
  return true;
}

bool PropulsionController::executeTrajectory(Vector2D* points, int num_points, float speed) {
  if (m_state != State::Stopped) {
    return false;
  }

  m_trajectory_buffer.push_segment(points, num_points);

  m_speed_controller.setAccelerationLimits(m_accel, m_deccel);
  m_speed_controller.setParameterRange(0, m_trajectory_buffer.max_parameter());
  m_speed_controller.setRequestedSpeed(speed);
  m_speed_controller.setFinalSpeed(m_reposition_distance != 0 ? m_reposition_speed : 0);
  m_speed_controller.reset(0, 0, 0);

  // Compute direction by taking scalar product of current robot orientation vector with tangent of
  // trajectory at origin
  auto target_point = m_trajectory_buffer.compute_point(0);
  float ux = cosf(m_target_pose.yaw);
  float uy = sinf(m_target_pose.yaw);
  m_direction = ux * target_point.tangent.x + uy * target_point.tangent.y > 0 ? Direction::Forward
                                                                              : Direction::Backward;



  setState(State::FollowTrajectory);
  return true;
};

bool PropulsionController::executePointTo(Vector2D point, float speed) {
  float diff_x = (point.x - m_current_pose.position.x);
  float diff_y = (point.y - m_current_pose.position.y);
  float target_yaw = atan2f(diff_y, diff_x);
  return executeRotation(angleDiff(target_yaw, m_target_pose.yaw), speed);
};

bool PropulsionController::executePointToBack(Vector2D point, float speed) {
  float diff_x = (point.x - m_current_pose.position.x);
  float diff_y = (point.y - m_current_pose.position.y);
  float target_yaw = atan2f(diff_y, diff_x) + c_pi;
  return executeRotation(angleDiff(target_yaw, m_target_pose.yaw), speed);
};

bool PropulsionController::executeFaceDirection(float direction, float yaw_rate) {
  return executeRotation(angleDiff(direction, m_target_pose.yaw), yaw_rate);
}

bool PropulsionController::executeMoveTo(Vector2D point, float speed) {
  Vector2D traj[2];
  traj[0] = m_target_pose.position;
  traj[1] = point;
  return executeTrajectory(traj, 2, speed);
};

bool PropulsionController::executeRotation(float delta_yaw, float yaw_rate) {
  if (m_state != State::Stopped) {
    return false;
  }
  // Compute yaw ramp to go to target_angle from current target angle
  m_begin_yaw = m_target_pose.yaw;

  // The speed controller output an increasing parameter.
  m_rotation_direction = delta_yaw >= 0 ? 1.0f : -1.0f;
  m_speed_controller.setAccelerationLimits(m_angular_accel, m_angular_deccel);
  m_speed_controller.setParameterRange(0, fabs(delta_yaw));
  m_speed_controller.setRequestedSpeed(yaw_rate);
  m_speed_controller.setFinalSpeed(0);
  m_speed_controller.reset(0, 0, 0);

  // Configure low level controller
  m_low_level_controller.setPidConfig(m_config.pid_configs[0]);
  m_low_level_controller.m_longi_control_level = 2;
  m_low_level_controller.m_yaw_control_level = 2;

  m_low_level_controller.m_left_motor_torque_lim = m_config.static_torque_limit;
  m_low_level_controller.m_right_motor_torque_lim = m_config.static_torque_limit;

  setState(State::Rotate);
  m_command_finished = false;
  return true;
}

void PropulsionController::prepareReposition(float distance, float speed) {
  m_reposition_distance = distance;
  m_reposition_speed = speed;
}
bool PropulsionController::executeRepositioning(float distance, float speed) {
  if (m_state != State::Stopped) {
    return false;
  }
  m_reposition_distance = distance;
  m_reposition_speed = speed;

  m_low_level_controller.m_motor_velocity_limit = m_config.cruise_pwm_limit;
  m_low_level_controller.m_left_motor_torque_lim = m_config.cruise_torque_limit;
  m_low_level_controller.m_right_motor_torque_lim = m_config.cruise_torque_limit;

  setState(State::Reposition);
  return true;
}

bool PropulsionController::executeTranslation(float distance, float speed) {
  Vector2D target;
  float ux = cos(m_target_pose.yaw);
  float uy = sin(m_target_pose.yaw);
  target.x = m_target_pose.position.x + ux * distance;
  target.y = m_target_pose.position.y + uy * distance;
  return executeMoveTo(target, speed);
}

void PropulsionController::enterManualControl() {
  m_state = State::ManualControl;
  m_state_changed = true;
}

void PropulsionController::exitManualControl() {
  m_current_pose = m_odometry->pose();
  m_target_pose = m_current_pose;
  setState(State::Stopped);
}

messages::PropulsionTelemetry PropulsionController::getTelemetry() const {
  messages::PropulsionTelemetry msg;
  msg.x = (int16_t)(m_current_pose.position.x * 4e3f);
  msg.y = (int16_t)(m_current_pose.position.y * 4e3f);
  msg.yaw = (int16_t)(m_current_pose.yaw * 32767 / c_pi);
  msg.speed = (int16_t)(m_current_pose.speed * 1000);
  msg.yaw_rate = (int16_t)(m_current_pose.yaw_rate * 1000);
  msg.acceleration = (int16_t)(m_current_pose.acceleration * 1000);
  msg.angular_acceleration = (int16_t)(m_current_pose.angular_acceleration * 1000);
  msg.left_encoder = m_odometry->leftEncoderValue();
  msg.right_encoder = m_odometry->rightEncoderValue();
  msg.left_pwm = (int8_t)(leftMotorVelocityInput() * 100);
  msg.right_pwm = (int8_t)(rightMotorVelocityInput() * 100);
  msg.state = (uint8_t)(state());
  msg.error = (uint8_t)(error());
  return msg;
}

messages::PropulsionTelemetryEx PropulsionController::getTelemetryEx() const {
  messages::PropulsionTelemetryEx msg;
  msg.target_x = (int16_t)(m_target_pose.position.x * 4e3f);
  msg.target_y = (int16_t)(m_target_pose.position.y * 4e3f);
  msg.target_yaw = (int16_t)(m_target_pose.yaw * 32767 / c_pi);
  msg.target_speed = (int16_t)(m_target_pose.speed * 1000);
  msg.target_yaw_rate = (int16_t)(m_target_pose.yaw_rate * 1000);
  msg.lookahead_x = (int16_t)(m_lookahead_position.x * 4e3f);
  msg.lookahead_y = (int16_t)(m_lookahead_position.y * 4e3f);
  msg.longitudinal_error = (int16_t)(m_low_level_controller.m_longi_error * 4e3f);
  msg.lateral_error = (int16_t)(m_low_level_controller.m_lateral_error * 4e3f);
  msg.speed_error = (int16_t)(m_low_level_controller.m_lateral_error * 1e3f);
  msg.yaw_error = (int16_t)(m_low_level_controller.m_yaw_error * 32767 / c_pi);

  return msg;
}
