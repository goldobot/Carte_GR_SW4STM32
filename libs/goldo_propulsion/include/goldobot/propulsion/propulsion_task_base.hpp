#pragma once
#include "goldobot/propulsion/controller.hpp"
#include "goldobot/propulsion/simple_odometry.hpp"
#include "goldobot/propulsion/robot_simulator.hpp"

#include <cstdint>

namespace goldobot {



struct ScopeConfig {
  uint16_t period;
  uint16_t num_channels;
  ScopeChannelConfig channels[8];
};

class PropulsionTaskBase

 public:
  PropulsionTask();
  const char* name() const override;

  SimpleOdometry& odometry();
  PropulsionController& controller();

  void setTaskConfig(const Config& config);
  void setControllerConfig(const PropulsionControllerConfig& config);
  void setOdometryConfig(const OdometryConfig& config);
  void setRobotSimulatorConfig(const RobotSimulatorConfig& config);

 private:
  MessageQueue m_message_queue;
  MessageQueue m_urgent_message_queue;
  MessageQueue m_odrive_message_queue;

  SimpleOdometry m_odometry;
  PropulsionController m_controller;
  RobotSimulator m_robot_simulator;

  bool m_use_simulator{false};

  Config m_config;

  uint16_t m_encoder_left{0};
  uint16_t m_encoders_right{0};
  uint32_t m_current_timestamp{0};

  float m_left_vel_setpoint{0};
  float m_right_vel_setpoint{0};

  uint32_t m_next_telemetry_ts{0};
  uint32_t m_next_telemetry_ex_ts{0};
  uint32_t m_next_pose_ts{0};
  uint32_t m_next_odrive_status_ts{0};
  uint32_t m_next_statistics_ts{0};

  Statistics m_statistics;

  uint16_t m_current_command_sequence_number{0};
  bool m_is_executing_command{false};

  void doStep();
  void processMessage();
  void processODriveMessage();
  void processUrgentMessage();
  void taskFunction() override;

  void onMsgExecuteTranslation(size_t msg_size);
  void onMsgExecuteRotation(size_t msg_size);
  void onMsgExecuteFaceDirection(size_t msg_size);
  void onMsgExecutePointTo(size_t msg_size);
  void onMsgExecutePointToBack(size_t msg_size);
  void onMsgExecuteMoveTo(size_t msg_size);
  void onMsgExecuteTrajectory(size_t msg_size);
  void onMsgExecuteReposition(size_t msg_size);

  void onMsgExecuteSetTargetPose(size_t msg_size);
  void onMsgExecuteMeasureNormal(size_t msg_size);

  void clearCommandQueue();

  void measureNormal(float angle, float distance);
  void measurePointLongi(Vector2D point, float sensor_offset);

  void setMotorsEnable(bool enable);
  void setMotorsPwm(float left_pwm, float right_pwm);
  void setMotorsTorqueLimits(float left, float right);
  void setSimulationMode(bool enable);

  void sendTelemetryMessages();
  void sendODriveStatus();
  void sendStatistics();

  uint16_t readCommand(MessageQueue& queue, void* buff, const size_t& size) {
    size_t size_ = size;
    return readCommand(queue, buff, size_);
  }
  uint16_t readCommand(MessageQueue& queue, void* buff, size_t& size);

  void sendCommandEvent(uint16_t sequence_number, CommandEvent event);
  void onCommandBegin(uint16_t sequence_number);
  void onCommandEnd();
  void onCommandCancel(uint16_t sequence_number);

  float scopeGetVariable(ScopeVariable type);
  void updateScope();
  void resetScope();
  void scopePush(float val, ScopeVariableEncoding encoding);

  ScopeConfig m_scope_config;
  uint8_t m_scope_total_size{0};
  uint8_t m_scope_buffer[64];  // uint32 packet timestamp followed by values
  size_t m_scope_idx{0};
  uint32_t m_next_scope_ts{0};

  ODriveClient m_odrive_client;

  // Odometry full encoders stream through secondary uart
  void updateOdometryStream(uint16_t left, uint16_t right);
  int m_odometry_stream_cnt{0};
  // each packet is 32 bits timestamp + 20 (left encoder, right encoder) values
  unsigned char m_odometry_stream_buffer[4 * (20 + 1)];

  // Odrive sent commands and received telemetry stream
  void updateOdriveStream();
  int m_odrive_stream_cnt{0};
  // Each packet is 8 bits var id, 8 bits timestamp, 32 bits value
  unsigned char m_odrive_stream_buffer[4 + 6 * 20];

  static unsigned char s_message_queue_buffer[1024];
  static unsigned char s_urgent_message_queue_buffer[1024];
  static unsigned char s_odrive_message_queue_buffer[256];
  static unsigned char exec_traj_buff[256];  // > 12 for traj params + 16*8 for points = 134
  static unsigned char s_scratchpad[512];
};
}  // namespace goldobot