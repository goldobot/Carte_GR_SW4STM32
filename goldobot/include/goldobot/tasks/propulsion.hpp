#pragma once
#include "goldobot/core/message_queue.hpp"
#include "goldobot/odrive/odrive_client.hpp"
#include "goldobot/platform/task.hpp"
#include "goldobot/propulsion/controller.hpp"
#include "goldobot/propulsion/simple_odometry.hpp"
#include "goldobot/propulsion/robot_simulator.hpp"

#include <cstdint>

namespace goldobot {

enum class ScopeVariable : uint16_t {
  None,
  PoseX,
  PoseY,
  PoseYaw,
  PoseSpeed,
  PoseYawRate,
  TargetX,
  TargetY,
  TargetYaw,
  TargetSpeed,
  TargetYawRate,
  LongiError,
  YawError,
  SpeedError,
  YawRateError,
  LeftMotorVelocitySetpoint,
  RightMotorVelocitySetpoint,
  ODriveAxis0VelEstimate,
  ODriveAxis1VelEstimate,
  ODriveAxis0CurrentIqSetpoint,
  ODriveAxis1CurrentIqSetpoint,
  ODriveVBus
};

enum class ScopeVariableEncoding : uint16_t {
  Raw8 = 0,
  Raw16 = 1,
  Raw32 = 2,
  Scaled8 = 4,
  Scaled16 = 5,
  Scaled32 = 6,
  Float = 8
};

struct ScopeChannelConfig {
  ScopeVariable variable{ScopeVariable::None};
  ScopeVariableEncoding encoding{ScopeVariableEncoding::Raw8};
  float min_value;
  float max_value;
};

struct ScopeConfig {
  uint16_t period;
  uint16_t num_channels;
  ScopeChannelConfig channels[8];
};

class PropulsionTask : public Task {
 public:
  enum class MotorControllerType : uint8_t { None, Pwm, ODriveUART };
  enum class CommandEvent : uint8_t { Begin = 0, End, Error, Cancel };

  struct Config {
    MotorControllerType motor_controller_type{MotorControllerType::None};
    uint8_t update_period_ms{1};
    uint8_t telemetry_period_ms{10};
    uint8_t telemetry_ex_period_ms{20};
    uint8_t pose_period_ms{50};
    uint8_t odrive_status_period_ms{100};
  };

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

  SimpleOdometry m_odometry;
  PropulsionController m_controller;
  RobotSimulator m_robot_simulator;

  bool m_use_simulator{false};

  Config m_config;

  uint16_t m_encoder_left{0};
  uint16_t m_encoders_right{0};

  float m_left_vel_setpoint{0};
  float m_right_vel_setpoint{0};

  uint32_t m_next_telemetry_ts{0};
  uint32_t m_next_telemetry_ex_ts{0};
  uint32_t m_next_pose_ts{0};
  uint32_t m_next_odrive_status_ts{0};

  uint16_t m_current_command_sequence_number{0};
  bool m_is_executing_command{false};

  uint32_t m_cycles_max{0};

  void doStep();
  void processMessage();
  void processUrgentMessage();
  void taskFunction() override;

  void onMsgExecuteTranslation(size_t msg_size);
  void onMsgExecuteRotation(size_t msg_size);
  void onMsgExecuteFaceDirection(size_t msg_size);
  void onMsgExecutePointTo(size_t msg_size);
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
  void setSimulationMode(bool enable);

  void sendTelemetryMessages();
  void sendODriveStatus();

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

  static unsigned char s_message_queue_buffer[1024];
  static unsigned char s_urgent_message_queue_buffer[1024];
  static unsigned char exec_traj_buff[256];  // > 12 for traj params + 16*8 for points = 134
};
}  // namespace goldobot
