#pragma once
#include "goldobot/core/message_queue.hpp"
#include "goldobot/odrive/odrive_client.hpp"
#include "goldobot/platform/task.hpp"
#include "goldobot/propulsion/controller.hpp"
#include "goldobot/propulsion/simple_odometry.hpp"
#include "goldobot/propulsion/robot_simulator.hpp"

#include <cstdint>

namespace goldobot {
class PropulsionTask : public Task {
	public:
	enum class MotorControllerType : uint8_t {
		None,
		Pwm,
		ODriveUART
	};

	struct Config
	{
		MotorControllerType motor_controller_type{MotorControllerType::None};
		uint8_t update_period_ms{1};
		uint8_t telemetry_period_ms{10};
		uint8_t telemetry_ex_period_ms{20};
		uint8_t pose_period_ms{50};
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

  uint32_t m_next_telemetry_ts{0};
  uint32_t m_next_telemetry_ex_ts{0};
  uint32_t m_next_pose_ts{0};
  uint32_t m_next_watchdog_ts{0};
  uint16_t m_current_command_sequence_number{0};

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
