#pragma once
#include "goldobot/platform/message_queue.hpp"
#include "goldobot/platform/task.hpp"
#include "goldobot/propulsion/controller.hpp"
#include "goldobot/propulsion/simple_odometry.hpp"
#include "goldobot/robot_simulator.hpp"

#include <cstdint>

namespace goldobot {
class PropulsionTask : public Task {
 public:
  PropulsionTask();
  const char* name() const override;

  SimpleOdometry& odometry();
  PropulsionController& controller();

 private:
  MessageQueue m_message_queue;
  MessageQueue m_urgent_message_queue;

  SimpleOdometry m_odometry;
  PropulsionController m_controller;
  RobotSimulator m_robot_simulator;

  bool m_use_simulator{false};

  uint16_t m_encoder_left{0};
  uint16_t m_encoders_right{0};
  uint16_t m_telemetry_counter{0};
  PropulsionController::State m_previous_state{PropulsionController::State::Inactive};

  bool m_adversary_detection_enabled{true};
  bool m_recalage_goldenium_armed{false};

  uint16_t m_odrive_seq{4096};
  static constexpr uint16_t c_odrive_key{0x9b40};  // firmware 5.1
  static constexpr uint16_t c_odrive_endpoint_input_vel[2] = {195, 424};
  static constexpr uint16_t c_odrive_endpoint_requested_state[2] = {74, 303};
  static constexpr uint16_t c_odrive_endpoint_control_mode[2] = {208, 437};
  static constexpr uint32_t c_odrive_consts_axis_state[2] = {1, 8};  // idle, closed_loop
  static constexpr uint32_t c_odrive_consts_control_mode = 2;        // velocity control
  uint16_t m_odrive_cnt{0};                                          // send one message every 10 ms

  void doStep();
  void processMessage();
  void processUrgentMessage();
  void taskFunction() override;

  void onMsgExecuteTrajectory();
  void onMsgExecutePointTo();

  void measureNormal(float angle, float distance);
  void measurePointLongi(Vector2D point, float sensor_offset);

  void setMotorsEnable(bool enable);
  void setMotorsPwm(float left_pwm, float right_pwm, bool immediate = false);

  void ODriveSetMotorsEnable(bool enable);
  void ODriveSetVelocitySetPoint(int axis, float vel_setpoint, float current_feedforward);

  template <typename T>
  void ODriveWriteEndpoint(uint16_t endpoint, T val);

  void sendTelemetryMessages();

  static unsigned char s_message_queue_buffer[1024];
  static unsigned char s_urgent_message_queue_buffer[1024];
};
}  // namespace goldobot
