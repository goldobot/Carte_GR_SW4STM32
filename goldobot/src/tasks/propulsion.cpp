#include "goldobot/tasks/propulsion.hpp"

#include "goldobot/hal.hpp"
#include "goldobot/robot.hpp"

#include "goldobot/core/math_utils.hpp"

#include <cassert>

// test, measure task running time
#include "stm32f3xx_hal.h"
#include "core_cm4.h"

using namespace goldobot;

bool goldo_hal_read_encoders(uint16_t& left, uint16_t& right);

unsigned char __attribute__((section(".ccmram"))) PropulsionTask::s_message_queue_buffer[1024];
unsigned char __attribute__((section(".ccmram")))
PropulsionTask::s_urgent_message_queue_buffer[1024];

unsigned char PropulsionTask::exec_traj_buff[256];
unsigned char PropulsionTask::s_scratchpad[512];

PropulsionTask::PropulsionTask()
    : m_message_queue(s_message_queue_buffer, sizeof(s_message_queue_buffer)),
      m_urgent_message_queue(s_urgent_message_queue_buffer, sizeof(s_urgent_message_queue_buffer)),
      m_controller(&m_odometry)

{}

const char* PropulsionTask::name() const { return "propulsion"; }

void PropulsionTask::setTaskConfig(const Config& config) { m_config = config; }

void PropulsionTask::setControllerConfig(const PropulsionControllerConfig& config) {
  m_controller.setConfig(config);
}

void PropulsionTask::setOdometryConfig(const OdometryConfig& config) {
  m_odometry.setConfig(config);
}

void PropulsionTask::setRobotSimulatorConfig(const RobotSimulatorConfig& config) {
  m_robot_simulator.m_config = config;
}

void PropulsionTask::doStep() {
  // test stopwatch
  uint32_t cyccnt_begin = DWT->CYCCNT;
  auto current_time = hal::get_tick_count();

  // Process urgent messages
  while (m_urgent_message_queue.message_ready()) {
    processUrgentMessage();
  }

  // Process command messages
  while (m_message_queue.message_ready() &&
         m_controller.state() == PropulsionController::State::Stopped) {
    processMessage();
  }

  uint32_t dwt = DWT->CYCCNT;
  m_cycle_time = dwt - m_last_sample_time;
  m_last_sample_time = dwt;
  // Update odometry
  if (m_use_simulator) {
    uint16_t left = m_robot_simulator.encoderLeft();
    uint16_t right = m_robot_simulator.encoderRight();
    m_odometry.update(left, right);
  } else {
	  uint16_t left;
	  uint16_t right;
	  while(goldo_hal_read_encoders(left, right))
	  {
		  m_odometry.update(left, right);
	  }
  }

  // run propulsion controller
  m_controller.update();

  // Send command status
  if (m_controller.commandFinished()) {
    onCommandEnd();
    if (m_message_queue.message_ready()) {
      processMessage();
    }
  }

  // check error

  // Check state change
  if (m_controller.stateChanged()) {
    uint8_t buff[2];
    buff[0] = (uint8_t)m_controller.state();
    buff[1] = (uint8_t)m_controller.error();
    Robot::instance().mainExchangeIn().pushMessage(CommMessageType::PropulsionState,
                                                   (unsigned char*)buff, 2);
  }

  if (m_controller.state() != PropulsionController::State::Inactive) {
    setMotorsPwm(m_controller.leftMotorVelocityInput(), m_controller.rightMotorVelocityInput());
  }

  if (m_config.motor_controller_type == MotorControllerType::ODriveUART) {
    m_odrive_client.doStep(hal::get_tick_count());
    auto& telemetry = m_odrive_client.telemetry();
    float torque_constant = 0.04;
    m_controller.setMotorsVelEstimates(-telemetry.axis[0].vel_estimate, telemetry.axis[1].vel_estimate);
    m_controller.setMotorsTorqueEstimates(-telemetry.axis[0].current_iq_setpoint * torque_constant, telemetry.axis[1].current_iq_setpoint * torque_constant);

    if (current_time >= m_next_odrive_status_ts) {
  	    sendODriveStatus();
  	  m_next_odrive_status_ts =
  	          std::max(m_next_odrive_status_ts + m_config.odrive_status_period_ms, current_time);
    }
  }

  if (m_use_simulator) {
    m_robot_simulator.doStep();
  }

  sendTelemetryMessages();
  updateScope();

  uint32_t cyccnt_end = DWT->CYCCNT;
  uint32_t cycles_count = cyccnt_end - cyccnt_begin;
  m_cycles_max = std::max(m_cycles_max, cycles_count);
}

void PropulsionTask::sendTelemetryMessages() {
  uint32_t current_time = hal::get_tick_count();

  if (current_time >= m_next_telemetry_ts) {
    auto msg = m_controller.getTelemetry();
    Robot::instance().mainExchangeOut().pushMessage(CommMessageType::PropulsionTelemetry,
                                                    (unsigned char*)&msg, sizeof(msg));
    m_next_telemetry_ts =
        std::max(m_next_telemetry_ts + m_config.telemetry_period_ms, current_time);
  }

  if (current_time >= m_next_telemetry_ex_ts) {
    auto msg = m_controller.getTelemetryEx();
    Robot::instance().mainExchangeOut().pushMessage(CommMessageType::PropulsionTelemetryEx,
                                                    (unsigned char*)&msg, sizeof(msg));
    m_next_telemetry_ex_ts =
        std::max(m_next_telemetry_ex_ts + m_config.telemetry_ex_period_ms, current_time);
  }

  if (current_time >= m_next_pose_ts) {
    float msg[3];
    msg[0] = m_odometry.pose().position.x;
    msg[1] = m_odometry.pose().position.y;
    msg[2] = m_odometry.pose().yaw;

    Robot::instance().mainExchangeOut().pushMessage(CommMessageType::PropulsionPose,
                                                    (unsigned char*)&msg, sizeof(msg));

    m_next_pose_ts = std::max(m_next_pose_ts + m_config.pose_period_ms, current_time);
  }
}

void PropulsionTask::processMessage() {
  auto message_type = (CommMessageType)m_message_queue.message_type();
  auto msg_size = m_message_queue.message_size();
  if (msg_size > sizeof(exec_traj_buff)) {
    msg_size = sizeof(exec_traj_buff);
  }

  m_message_queue.pop_message(exec_traj_buff, msg_size);
  uint16_t sequence_number = *(uint16_t*)exec_traj_buff;

  switch (message_type) {
    case CommMessageType::PropulsionExecuteTrajectory:
      onMsgExecuteTrajectory(msg_size);
      break;
    case CommMessageType::PropulsionExecuteRotation:
      onMsgExecuteRotation(msg_size);
      break;
    case CommMessageType::PropulsionExecuteTranslation:
      onMsgExecuteTranslation(msg_size);
      break;
    case CommMessageType::PropulsionExecutePointTo:
      onMsgExecutePointTo(msg_size);
      break;
    case CommMessageType::PropulsionExecuteFaceDirection:
      onMsgExecuteFaceDirection(msg_size);
      break;
    case CommMessageType::PropulsionExecuteMoveTo:
      onMsgExecuteMoveTo(msg_size);
      break;
    case CommMessageType::PropulsionExecuteReposition:
      onMsgExecuteReposition(msg_size);
      break;
      // case CommMessageType::PropulsionEnterManualControl:
      //  m_controller.enterManualControl();
      //  break;
      // case CommMessageType::PropulsionExitManualControl:
      //  m_controller.exitManualControl();
      //  break;
      // case CommMessageType::PropulsionSetControlLevels: {
      //  uint8_t buff[2];
      // m_message_queue.pop_message((unsigned char*)buff, 2);
      // m_controller.setControlLevels(buff[0], buff[1]);
      // } break;
    case CommMessageType::PropulsionSetTargetPose:
      onMsgExecuteSetTargetPose(msg_size);
      break;
    case CommMessageType::PropulsionMeasureNormal:
      onMsgExecuteMeasureNormal(msg_size);
      break;
    default:
      break;
  }
  onCommandBegin(sequence_number);
}

void PropulsionTask::processUrgentMessage() {
  auto message_type = (CommMessageType)m_urgent_message_queue.message_type();
  auto message_size = m_urgent_message_queue.message_size();

  uint32_t current_time = hal::get_tick_count();

  switch (message_type) {
    case CommMessageType::ODriveResponsePacket: {
      uint8_t buff[6];
      m_urgent_message_queue.pop_message((unsigned char*)&buff, 16);
      uint16_t seq = *(uint16_t*)buff & 0x3fff;
      m_odrive_client.processResponse(current_time, seq, buff + 2, message_size - 2);
    } break;
    case CommMessageType::PropulsionSetPose: {
      float pose[3];
      m_urgent_message_queue.pop_message((unsigned char*)&pose, 12);
      m_controller.resetPose(pose[0], pose[1], pose[2]);
    } break;
    case CommMessageType::OdometryConfigGet: {
      auto config = m_odometry.config();
      Robot::instance().mainExchangeOut().pushMessage(CommMessageType::OdometryConfigGetStatus,
                                                      (unsigned char*)&config, sizeof(config));
      m_urgent_message_queue.pop_message(nullptr, 0);
    } break;
    case CommMessageType::OdometryConfigSet: {
      OdometryConfig config;
      m_urgent_message_queue.pop_message((unsigned char*)&config, sizeof(config));
      m_odometry.setConfig(config);
    } break;
    case CommMessageType::PropulsionConfigGet: {
      auto config = m_controller.config();
      Robot::instance().mainExchangeOut().pushMessage(CommMessageType::PropulsionConfigGetStatus,
                                                      (unsigned char*)&config, sizeof(config));
      m_urgent_message_queue.pop_message(nullptr, 0);
    } break;
    case CommMessageType::PropulsionConfigSet: {
      PropulsionControllerConfig config;
      m_urgent_message_queue.pop_message((unsigned char*)&config, sizeof(config));
      m_controller.setConfig(config);
    } break;
    case CommMessageType::PropulsionSetTargetSpeed: {
      uint16_t sequence_number;
      float target_speed;

      unsigned char* buffs[] = {(unsigned char*)&sequence_number, (unsigned char*)&target_speed};
      size_t sizes [] = {2,4};

      m_urgent_message_queue.pop_message(buffs, sizes, 2);

      m_controller.setTargetSpeed(target_speed);
      sendCommandEvent(sequence_number, CommandEvent::End);
    } break;
    case CommMessageType::PropulsionSetAccelerationLimits: {
      float params[4];
      m_urgent_message_queue.pop_message((unsigned char*)&params, sizeof(params));
      m_controller.setAccelerationLimits(params[0], params[1], params[2], params[3]);
    } break;
    case CommMessageType::PropulsionEmergencyStop:
      m_controller.emergencyStop();
      m_urgent_message_queue.pop_message(nullptr, 0);
      break;
    case CommMessageType::PropulsionClearError:
      m_controller.clearError();
      m_urgent_message_queue.pop_message(nullptr, 0);
      break;
    case CommMessageType::PropulsionClearCommandQueue:
      m_urgent_message_queue.pop_message(nullptr, 0);
      clearCommandQueue();
      break;
    case CommMessageType::PropulsionEnableSet: {
      uint8_t enabled;
      m_urgent_message_queue.pop_message((unsigned char*)&enabled, 1);
      m_controller.setEnable(enabled);
      if (!enabled) {
        clearCommandQueue();
      }
    } break;
    case CommMessageType::PropulsionSetSimulationMode: {
      uint8_t enable;
      m_urgent_message_queue.pop_message((unsigned char*)&enable, 1);
      setSimulationMode(enable);
    } break;
    case CommMessageType::PropulsionScopeConfig: {
      m_urgent_message_queue.pop_message((unsigned char*)&m_scope_config, sizeof(m_scope_config));
      resetScope();
    } break;
    case CommMessageType::PropulsionMotorsEnableSet: {
      uint8_t enabled;
      m_urgent_message_queue.pop_message((unsigned char*)&enabled, 1);
      setMotorsEnable(enabled);
    } break;
    case CommMessageType::PropulsionMotorsVelocitySetpointsSet: {
      float pwm[4];
      m_urgent_message_queue.pop_message((unsigned char*)&pwm, 16);
      setMotorsPwm(pwm[0], pwm[1]);
    } break;
    case CommMessageType::PropulsionMotorsTorqueLimitsSet: {
          float pwm[2];
          m_urgent_message_queue.pop_message((unsigned char*)s_scratchpad, 10);
          memcpy(pwm, s_scratchpad + 2, 8);
          uint16_t sequence_number = *reinterpret_cast<uint16_t*>(s_scratchpad);
          setMotorsTorqueLimits(pwm[0], pwm[1]);
          sendCommandEvent(sequence_number, CommandEvent::End);
        } break;
      /* case CommMessageType::PropulsionMeasurePoint: {
         float buff[4];
         m_urgent_message_queue.pop_message((unsigned char*)&buff, sizeof(buff));
         m_odometry.measurePerpendicularPoint(buff[0], buff[1], *(Vector2D*)(buff + 2));
         auto pose = m_odometry.pose();
         // Set controller to new pose
         m_controller.resetPose(pose.position.x, pose.position.y, pose.yaw);
       } break;*/
    default:
      m_urgent_message_queue.pop_message(nullptr, 0);
      break;
  }
}

// Command messages
void PropulsionTask::onMsgExecuteReposition(size_t msg_size) {
  float speed = *(float*)(exec_traj_buff + 2);
  float accel = *(float*)(exec_traj_buff + 6);
  m_controller.executeRepositioning(speed, accel);
}

void PropulsionTask::onMsgExecuteSetTargetPose(size_t msg_size) {
  RobotPose pose = *(RobotPose*)(exec_traj_buff + 2);
  m_controller.setTargetPose(pose);
}

void PropulsionTask::onMsgExecuteMeasureNormal(size_t msg_size) {
  float angle = *(float*)(exec_traj_buff + 2);
  float distance = *(float*)(exec_traj_buff + 6);
  measureNormal(angle, distance);
}

void PropulsionTask::onMsgExecuteTranslation(size_t msg_size) {
  float distance = *(float*)(exec_traj_buff + 2);
  float speed = *(float*)(exec_traj_buff + 6);
  m_controller.executeTranslation(distance, speed);
}

void PropulsionTask::onMsgExecuteRotation(size_t msg_size) {
  float angle = *(float*)(exec_traj_buff + 2);
  float yaw_rate = *(float*)(exec_traj_buff + 6);
  m_controller.executeRotation(angle, yaw_rate);
}

void PropulsionTask::onMsgExecuteMoveTo(size_t msg_size) {
  Vector2D point = *(Vector2D*)(exec_traj_buff + 2);
  float speed = *(float*)(exec_traj_buff + 10);
  m_controller.executeMoveTo(point, speed);
}

void PropulsionTask::onMsgExecuteFaceDirection(size_t msg_size) {
  float yaw = *(float*)(exec_traj_buff + 2);
  float yaw_rate = *(float*)(exec_traj_buff + 6);
  m_controller.executeFaceDirection(yaw, yaw_rate);
}

void PropulsionTask::onMsgExecutePointTo(size_t msg_size) {
  Vector2D point = *(Vector2D*)(exec_traj_buff + 2);
  float yaw_rate = *(float*)(exec_traj_buff + 10);
  m_controller.executePointTo(point, yaw_rate);
}

void PropulsionTask::onMsgExecuteTrajectory(size_t msg_size) {
  // todo: send error message if message size is too large
  if (msg_size <= 136) {
    // message has a header of  8 bytes: uint16 sequence number, padding and float speed. each point is 2
    // floats
    int num_points = (msg_size - 8) / 8;
    float speed = *(float*)(exec_traj_buff + 4);
    Vector2D* points = (Vector2D*)(exec_traj_buff + 8);
    m_controller.executeTrajectory(points, num_points, speed);
  }
}

SimpleOdometry& PropulsionTask::odometry() { return m_odometry; }

PropulsionController& PropulsionTask::controller() { return m_controller; }

void PropulsionTask::measureNormal(float angle, float distance) {
  auto pose = m_odometry.pose();
  Vector2D normal{cosf(angle), sinf(angle)};
  // Check if front or back is touching the border
  float dot = normal.x * cos(pose.yaw) + normal.y * sin(pose.yaw);
  if (dot > 0) {
    // border normal is aligned with robot yaw
    // means the robot back is touching the border
    distance = distance + Robot::instance().robotGeometry().back_length;
  } else {
    // touched on the front
    distance = distance + Robot::instance().robotGeometry().front_length;
  }
  // Project current position on line and adjust yaw
  m_odometry.measureLineNormal(normal, distance);
  pose = m_odometry.pose();
  // Set controller to new pose
  m_controller.resetPose(pose.position.x, pose.position.y, pose.yaw);
}

void PropulsionTask::setMotorsEnable(bool enable) {
  if (m_use_simulator) {
    m_robot_simulator.m_motors_enable = enable;
  } else {
    switch (m_config.motor_controller_type) {
      case MotorControllerType::ODriveUART:
        m_odrive_client.setMotorsEnable(enable);
        break;
      default:
        break;
    }
  }
}

void PropulsionTask::setSimulationMode(bool enable) {
  // switch to simulation mode
  if (m_use_simulator == false && enable == true) {
    setMotorsEnable(false);
    setMotorsPwm(0, 0);
    m_use_simulator = true;
    m_robot_simulator.m_left_encoder.m_counts = m_odometry.leftEncoderValue();
    m_robot_simulator.m_left_encoder.m_delta = 0;
    m_robot_simulator.m_right_encoder.m_counts = m_odometry.rightEncoderValue();
    m_robot_simulator.m_right_encoder.m_delta = 0;
  }
}

void PropulsionTask::setMotorsPwm(float left_pwm, float right_pwm) {
  m_left_vel_setpoint = left_pwm;
  m_right_vel_setpoint = right_pwm;
  if (m_use_simulator) {
    m_robot_simulator.m_left_pwm = left_pwm;
    m_robot_simulator.m_right_pwm = right_pwm;
  } else {
    switch (m_config.motor_controller_type) {
      case MotorControllerType::ODriveUART:
        m_odrive_client.setVelocitySetPoint(0, -left_pwm, 0);
        m_odrive_client.setVelocitySetPoint(1, right_pwm, 0);
        break;
      case MotorControllerType::Pwm:
        hal::pwm_set(0, left_pwm);
        hal::pwm_set(1, right_pwm);
        break;
      default:
        break;
    }
  }
}

void PropulsionTask::setMotorsTorqueLimits(float left, float right)
{
	if (m_use_simulator) {

	  } else {
	    switch (m_config.motor_controller_type) {
	      case MotorControllerType::ODriveUART:
	        m_odrive_client.setTorqueLimit(0, left);
	        m_odrive_client.setTorqueLimit(1, right);
	        break;
	      default:
	        break;
	    }
	  }
}

void PropulsionTask::sendCommandEvent(uint16_t sequence_number, CommandEvent event )
{
	  uint8_t buff[4];  // sequence_number, status, error
	  buff[2] = static_cast<uint8_t>(event);
	  buff[3] = static_cast<uint8_t>(m_controller.error());
	  *(uint16_t*)(buff) = sequence_number;
	  Robot::instance().mainExchangeOutPrio().pushMessage(CommMessageType::PropulsionCommandEvent, buff,
	                                                      4);
}

void PropulsionTask::onCommandBegin(uint16_t sequence_number) {
  m_current_command_sequence_number = sequence_number;

  uint8_t buff[4];  // sequence_number, status, error
  buff[2] = static_cast<uint8_t>(CommandEvent::Begin);
  buff[3] = static_cast<uint8_t>(m_controller.error());
  *(uint16_t*)(buff) = m_current_command_sequence_number;
  Robot::instance().mainExchangeOutPrio().pushMessage(CommMessageType::PropulsionCommandEvent, buff,
                                                      4);
  m_is_executing_command = true;
}

void PropulsionTask::onCommandEnd() {
  uint8_t buff[4];  // sequence_number, status, error
  buff[2] = static_cast<uint8_t>(m_controller.state() != PropulsionController::State::Error
                                     ? CommandEvent::End
                                     : CommandEvent::Error);
  buff[3] = static_cast<uint8_t>(m_controller.error());
  *(uint16_t*)(buff) = m_current_command_sequence_number;
  Robot::instance().mainExchangeOutPrio().pushMessage(CommMessageType::PropulsionCommandEvent, buff,
                                                      4);
  m_is_executing_command = false;
  if (m_controller.state() == PropulsionController::State::Error) {

  }
}

void PropulsionTask::onCommandCancel(uint16_t sequence_number) {
  m_current_command_sequence_number = sequence_number;

  uint8_t buff[4];  // sequence_number, status, error
  buff[2] = static_cast<uint8_t>(CommandEvent::Cancel);
  buff[3] = static_cast<uint8_t>(m_controller.error());
  *(uint16_t*)(buff) = m_current_command_sequence_number;
  Robot::instance().mainExchangeOutPrio().pushMessage(CommMessageType::PropulsionCommandEvent, buff,
                                                      4);
  m_is_executing_command = false;
}

void PropulsionTask::clearCommandQueue() {
  if (m_is_executing_command) {
    onCommandCancel(m_current_command_sequence_number);
  }

  while (m_message_queue.message_ready()) {
    m_message_queue.pop_message(exec_traj_buff, 2);
    uint16_t sequence_number = *(uint16_t*)exec_traj_buff;
    onCommandCancel(sequence_number);
  }
}

void PropulsionTask::sendODriveStatus()
{
	{
	auto axis_states = m_odrive_client.axisStates();
	Robot::instance().mainExchangeOut().pushMessage(CommMessageType::PropulsionODriveAxisStates, (unsigned char*)&axis_states,
	                                                      sizeof(axis_states));
	}
	{
		auto axis_errors = m_odrive_client.errors();
		Robot::instance().mainExchangeOut().pushMessage(CommMessageType::PropulsionODriveAxisErrors, (unsigned char*)&axis_errors,
			                                                      sizeof(axis_errors));
	}
	{
		auto statistics = m_odrive_client.statistics();
		Robot::instance().mainExchangeOut().pushMessage(CommMessageType::PropulsionODriveStatistics, (unsigned char*)&statistics,
				                                                      sizeof(statistics));
	}

	//const std::array<AxisCalibrationState, 2>& axisCalibrationStates() const noexcept;
}

float PropulsionTask::scopeGetVariable(ScopeVariable type) {
  switch (type) {
    case ScopeVariable::PoseX:
      return m_odometry.pose().position.x;
    case ScopeVariable::PoseY:
      return m_odometry.pose().position.y;
    case ScopeVariable::PoseYaw:
      return m_odometry.pose().yaw;
    case ScopeVariable::PoseSpeed:
      return m_odometry.pose().speed;
    case ScopeVariable::PoseYawRate:
      return m_odometry.pose().yaw_rate;
    case ScopeVariable::PoseAcceleration:
    	return m_odometry.pose().acceleration;
    case ScopeVariable::TargetX:
      return m_controller.targetPose().position.x;
    case ScopeVariable::TargetY:
      return m_controller.targetPose().position.y;
    case ScopeVariable::TargetYaw:
      return m_controller.targetPose().yaw;
    case ScopeVariable::TargetSpeed:
      return m_controller.targetPose().speed;
    case ScopeVariable::TargetYawRate:
      return m_controller.targetPose().yaw_rate;
    case ScopeVariable::TargetAcceleration:
    	return m_controller.targetPose().acceleration;
    case ScopeVariable::LeftMotorVelocitySetpoint:
      return m_left_vel_setpoint;
    case ScopeVariable::RightMotorVelocitySetpoint:
      return m_right_vel_setpoint;
    case ScopeVariable::LongiError:
      return m_controller.lowLevelController().m_longi_error;
    case ScopeVariable::YawError:
      return m_controller.lowLevelController().m_yaw_error;
    case ScopeVariable::SpeedError:
      return m_controller.lowLevelController().m_speed_error;
    case ScopeVariable::YawRateError:
      return m_controller.lowLevelController().m_yaw_rate_error;
    // ODrive telemetry
    case ScopeVariable::ODriveAxis0VelEstimate:
      return m_odrive_client.telemetry().axis[0].vel_estimate;
    case ScopeVariable::ODriveAxis1VelEstimate:
      return m_odrive_client.telemetry().axis[1].vel_estimate;
    case ScopeVariable::ODriveAxis0CurrentIqSetpoint:
      return m_odrive_client.telemetry().axis[0].current_iq_setpoint;
    case ScopeVariable::ODriveAxis1CurrentIqSetpoint:
      return m_odrive_client.telemetry().axis[1].current_iq_setpoint;
    case ScopeVariable::ODriveVBus:
      return m_odrive_client.m_odrv_requests.vbus;
    case ScopeVariable::ODriveIBus:
      return m_odrive_client.m_odrv_requests.ibus;
    case ScopeVariable::EncodersLeftCounts:
      return m_odometry.leftEncoderValue();
    case ScopeVariable::EncodersRightCounts:
      return m_odometry.rightEncoderValue();
    case ScopeVariable::BlockingDetectorSpeedEstimate:
    	return m_controller.m_blocking_detector.m_speed_estimate;
    case ScopeVariable::BlockingDetectorForceEstimate:
        	return m_controller.m_blocking_detector.m_force_estimate;
    default:
      return 0;
  }

  m_odrive_client.telemetry().axis[0].current_iq_setpoint;
}

void PropulsionTask::resetScope() {
  m_scope_total_size = 0;
  m_scope_idx = 0;
  if (m_scope_config.num_channels > 8) {
    m_scope_config.num_channels = 8;
  }

  for (unsigned i = 0; i < m_scope_config.num_channels; i++) {
    switch (m_scope_config.channels[i].encoding) {
      case ScopeVariableEncoding::Raw8:
        m_scope_total_size += 1;
        break;
      case ScopeVariableEncoding::Raw16:
        m_scope_total_size += 2;
        break;
      case ScopeVariableEncoding::Raw32:
        m_scope_total_size += 4;
        break;
      case ScopeVariableEncoding::Scaled8:
        m_scope_total_size += 1;
        break;
      case ScopeVariableEncoding::Scaled16:
        m_scope_total_size += 2;
        break;
      case ScopeVariableEncoding::Scaled32:
        m_scope_total_size += 4;
        break;
      case ScopeVariableEncoding::Float:
        m_scope_total_size += 4;
        break;
      default:
        break;
    }
  }
}

void PropulsionTask::scopePush(float val, ScopeVariableEncoding encoding) {
  uint8_t* ptr = &m_scope_buffer[m_scope_idx];
  switch (encoding) {
    case ScopeVariableEncoding::Scaled8:
      *reinterpret_cast<uint8_t*>(ptr) =
          static_cast<uint8_t>(val * std::numeric_limits<uint8_t>::max());
      m_scope_idx += 1;
      break;
    case ScopeVariableEncoding::Scaled16:
      *reinterpret_cast<uint16_t*>(ptr) =
          static_cast<uint8_t>(val * std::numeric_limits<uint16_t>::max());
      m_scope_idx += 2;
      break;
    case ScopeVariableEncoding::Scaled32:
      *reinterpret_cast<uint8_t*>(ptr) =
          static_cast<uint32_t>(val * std::numeric_limits<uint32_t>::max());
      m_scope_idx += 4;
      break;
    default:
      break;
  }
}

void PropulsionTask::updateScope() {
  if (m_scope_config.num_channels == 0) {
    return;
  }

  auto current_time = hal::get_tick_count();

  if (current_time < m_next_scope_ts) {
    return;
  }

  m_next_scope_ts = std::max(m_next_scope_ts + m_scope_config.period, current_time);

  if (m_scope_idx == 0) {
    *reinterpret_cast<uint16_t*>(&m_scope_buffer[m_scope_idx]) = (uint16_t)hal::get_tick_count();
    m_scope_idx = 2;
  }

  for (unsigned i = 0; i < m_scope_config.num_channels; i++) {
    const auto& chan = m_scope_config.channels[i];
    if (chan.encoding == ScopeVariableEncoding::Float) {
      float val = scopeGetVariable(chan.variable);
      uint8_t* ptr = &m_scope_buffer[m_scope_idx];
      *reinterpret_cast<float*>(ptr) = val;
      m_scope_idx += 4;
    } else {
      float val = scopeGetVariable(chan.variable);
      float val_normalized = (val - chan.min_value) / (chan.max_value - chan.min_value);
      val_normalized = clamp(val_normalized, 0.0f, 1.0f);
      scopePush(val_normalized, chan.encoding);
    }
  }
  if (m_scope_idx + m_scope_total_size >= sizeof(m_scope_buffer)) {
    Robot::instance().mainExchangeOut().pushMessage(CommMessageType::PropulsionScopeData,
                                                    m_scope_buffer, m_scope_idx);
    m_scope_idx = 0;
  }
}

void PropulsionTask::taskFunction() {
  // queued commands
  Robot::instance().mainExchangeIn().subscribe({140, 169, &m_message_queue});

  // immediate commands
  Robot::instance().mainExchangeIn().subscribe({100, 119, &m_urgent_message_queue});
  Robot::instance().mainExchangeIn().subscribe({151, 152, &m_urgent_message_queue});

  // immediate commands
  Robot::instance().exchangeInternal().subscribe({12, 12, &m_urgent_message_queue});

  // configs
  Robot::instance().mainExchangeIn().subscribe({210, 219, &m_urgent_message_queue});

  // orive responses
  Robot::instance().exchangeInternal().subscribe({51, 51, &m_urgent_message_queue});

  // Set task to high
  set_priority(4);

  // Setup odometry
  uint16_t left = hal::encoder_get(0);
  uint16_t right = hal::encoder_get(1);

  m_odometry.setPeriod(m_config.update_period_ms * 1e-3f);
  m_odometry.setConfig(m_odometry.config());
  m_odometry.reset(left, right);

  m_odrive_client.setOutputExchange(&Robot::instance().mainExchangeIn(),
                                    CommMessageType::ODriveRequestPacket);

  while (1) {
    checkStateUpdate();
    if (m_state == Running) {
      doStep();
    }
    delay_periodic(m_config.update_period_ms);
  }
}
