#pragma once
#include "goldobot/core/message_queue.hpp"
#include "goldobot/platform/task.hpp"
#include "goldobot/propulsion/controller.hpp"
#include "goldobot/propulsion/simple_odometry.hpp"
#include "goldobot/robot_simulator.hpp"
#include "goldobot/odrive/odrive_client.hpp"

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

  void doStep();
  void processMessage();
  void processUrgentMessage();
  void taskFunction() override;

  void onMsgExecuteTranslation();
  void onMsgExecuteRotation();
  void onMsgExecuteMoveTo();
  void onMsgExecutePointTo();
  void onMsgExecuteTrajectory();


  void measureNormal(float angle, float distance);
  void measurePointLongi(Vector2D point, float sensor_offset);

  void setMotorsEnable(bool enable);
  void setMotorsPwm(float left_pwm, float right_pwm, bool immediate = false);
  void setSimulationMode(bool enable);

  void sendTelemetryMessages();

  void ackCommand(uint16_t sequence_number);

  ODriveClient m_odrive_client;

  static unsigned char s_message_queue_buffer[1024];
  static unsigned char s_urgent_message_queue_buffer[1024];
};
}  // namespace goldobot
