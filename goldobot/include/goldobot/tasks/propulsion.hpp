#pragma once
#include <cstdint>

#include "goldobot/platform/message_queue.hpp"
#include "goldobot/platform/task.hpp"
#include "goldobot/propulsion/controller.hpp"
#include "goldobot/propulsion/simple_odometry.hpp"
#include "goldobot/robot_simulator.hpp"

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

  void doStep();
  void processMessage();
  void processUrgentMessage();
  void taskFunction() override;

  void onMsgExecuteTrajectory();
  void onMsgExecutePointTo();

  void measureNormal(float angle, float distance);
  void measurePointLongi(Vector2D point, float sensor_offset);

  void setMotorsPwm(float left_pwm, float right_pwm);

  void sendTelemetryMessages();

  static unsigned char s_message_queue_buffer[1024];
  static unsigned char s_urgent_message_queue_buffer[1024];
};
}  // namespace goldobot
