#include "goldobot/tasks/propulsion.hpp"

#include "goldobot/hal.hpp"
#include "goldobot/robot.hpp"

#include <cassert>

#ifndef M_PI
#define M_PI (3.14159265358979323846)
#endif

using namespace goldobot;

unsigned char __attribute__((section(".ccmram"))) PropulsionTask::s_message_queue_buffer[1024];
unsigned char __attribute__((section(".ccmram")))
PropulsionTask::s_urgent_message_queue_buffer[1024];

constexpr uint16_t PropulsionTask::c_odrive_endpoint_input_vel[2];

PropulsionTask::PropulsionTask()
    : m_message_queue(s_message_queue_buffer, sizeof(s_message_queue_buffer)),
      m_urgent_message_queue(s_urgent_message_queue_buffer, sizeof(s_urgent_message_queue_buffer)),
      m_controller(&m_odometry)

{}

const char* PropulsionTask::name() const { return "propulsion"; }

void PropulsionTask::doStep() {
  // Process urgent messages
  while (m_urgent_message_queue.message_ready()) {
    processUrgentMessage();
  }

  // Process messages
  while (m_message_queue.message_ready() &&
         m_controller.state() == PropulsionController::State::Stopped) {
    processMessage();
  }

  while (m_message_queue.message_ready() &&
         m_controller.state() == PropulsionController::State::ManualControl) {
    processMessage();
  }

  // Update odometry
  if (m_use_simulator) {
    uint16_t left = m_robot_simulator.encoderLeft();
    uint16_t right = m_robot_simulator.encoderRight();
    m_odometry.update(left, right);
  } else {
    uint16_t left = hal::encoder_get(0);
    uint16_t right = hal::encoder_get(1);
    m_odometry.update(left, right);
  }

  m_controller.update();

  while (m_message_queue.message_ready() &&
         m_controller.state() == PropulsionController::State::Stopped) {
    processMessage();
  }

  // Check state change
  if (m_controller.state() != m_previous_state) {
    uint8_t buff[2];
    buff[0] = (uint8_t)m_controller.state();
    buff[1] = (uint8_t)m_previous_state;
    m_previous_state = m_controller.state();
    /* Robot::instance().mainExchangeIn().pushMessage(CommMessageType::PropulsionStateChanged,
                                                    (unsigned char*)buff, 2);
     Robot::instance().mainExchangeOut().pushMessage(CommMessageType::PropulsionStateChanged,
                                                     (unsigned char*)buff, 2);*/
  }

  if (m_controller.state() != PropulsionController::State::Inactive) {
    setMotorsPwm(m_controller.leftMotorPwm(), m_controller.rightMotorPwm());
  }
  if (m_use_simulator) {
    m_robot_simulator.doStep();
  }

  sendTelemetryMessages();
}

void PropulsionTask::sendTelemetryMessages() {
  m_telemetry_counter++;
  if (m_telemetry_counter == 20) {
    auto msg = m_controller.getTelemetryEx();
    Robot::instance().mainExchangeOut().pushMessage(CommMessageType::PropulsionTelemetryEx,
                                                    (unsigned char*)&msg, sizeof(msg));
    m_telemetry_counter = 0;
  }

  if (m_telemetry_counter == 40) {
    float msg[3];
    msg[0] = m_odometry.pose().position.x;
    msg[1] = m_odometry.pose().position.y;
    msg[2] = m_odometry.pose().yaw;

    Robot::instance().mainExchangeOut().pushMessage(CommMessageType::PropulsionPose,
                                                    (unsigned char*)&msg, sizeof(msg));
    m_telemetry_counter = 0;
  }

  if (m_telemetry_counter % 5 == 0) {
    auto msg = m_controller.getTelemetry();
    Robot::instance().mainExchangeOut().pushMessage(CommMessageType::PropulsionTelemetry,
                                                    (unsigned char*)&msg, sizeof(msg));
  }
}

void PropulsionTask::processMessage() {
  auto message_type = (CommMessageType)m_message_queue.message_type();

  switch (message_type) {
    case CommMessageType::PropulsionExecuteTrajectory:
      onMsgExecuteTrajectory();
      break;
    case CommMessageType::PropulsionExecuteRotation: {
      float params[4];
      m_message_queue.pop_message((unsigned char*)&params, sizeof(params));
      m_controller.executeRotation(params[0], params[1], params[2], params[3]);
    } break;
    case CommMessageType::PropulsionExecuteTranslation: {
      float params[4];
      m_message_queue.pop_message((unsigned char*)&params, sizeof(params));
      m_controller.executeTranslation(params[0], params[1], params[2], params[3]);
    } break;
    case CommMessageType::PropulsionExecutePointTo:
      onMsgExecutePointTo();
      break;
    case CommMessageType::PropulsionExecuteFaceDirection: {
      float params[4];
      m_message_queue.pop_message((unsigned char*)&params, sizeof(params));
      m_controller.executeFaceDirection(params[0], params[1], params[2], params[3]);
    } break;
    case CommMessageType::PropulsionExecuteMoveTo: {
      float params[5];
      m_message_queue.pop_message((unsigned char*)&params, sizeof(params));
      m_controller.executeMoveTo(*(Vector2D*)(params), params[2], params[3], params[4]);
    } break;
    case CommMessageType::PropulsionExecuteReposition: {
      float params[2];
      m_message_queue.pop_message((unsigned char*)&params, sizeof(params));
      m_controller.executeRepositioning(params[0], params[1]);
    } break;
    case CommMessageType::PropulsionEnterManualControl:
      m_controller.enterManualControl();
      break;

    case CommMessageType::PropulsionExitManualControl:
      m_controller.exitManualControl();
      break;
    case CommMessageType::PropulsionSetControlLevels: {
      uint8_t buff[2];
      m_message_queue.pop_message((unsigned char*)buff, 2);
      m_controller.setControlLevels(buff[0], buff[1]);
    } break;
    case CommMessageType::PropulsionSetTargetPose: {
      RobotPose pose;
      m_message_queue.pop_message((unsigned char*)&pose, sizeof(pose));
      m_controller.setTargetPose(pose);
    } break;
    case CommMessageType::PropulsionMeasureNormal: {
      float buff[2];
      m_message_queue.pop_message((unsigned char*)&buff, sizeof(buff));
      measureNormal(buff[0], buff[1]);
    } break;
    default:
      m_message_queue.pop_message(nullptr, 0);
      break;
  }
}

void PropulsionTask::processUrgentMessage() {
  auto message_type = (CommMessageType)m_urgent_message_queue.message_type();

  switch (message_type) {
      /* case CommMessageType::PropulsionSetPose: {
         float pose[3];
         m_urgent_message_queue.pop_message((unsigned char*)&pose, 12);
         m_controller.resetPose(pose[0], pose[1], pose[2]);
       } break;*/
    /*case CommMessageType::PropulsionSetAdversaryDetectionEnable: {
      uint8_t buff;
      m_urgent_message_queue.pop_message((unsigned char*)&buff, 1);
      m_adversary_detection_enabled = (bool)buff;
    } break;*/
    case CommMessageType::OdometryConfigGet: {
      auto config = m_odometry.config();
      Robot::instance().mainExchangeOut().pushMessage(CommMessageType::OdometryConfigGetStatus,
                                                      (unsigned char*)&config, sizeof(config));
      m_urgent_message_queue.pop_message(nullptr, 0);
    } break;
    case CommMessageType::OdometryConfigSet: {
      OdometryConfig config;
      m_urgent_message_queue.pop_message((unsigned char*)&config, sizeof(config));
      m_odometry.setConfig(config, 1e-3f);
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
    /*case CommMessageType::CmdEmergencyStop:
      m_controller.emergencyStop();
      m_urgent_message_queue.pop_message(nullptr, 0);
      break;
    case CommMessageType::PropulsionClearError:
      m_controller.clearError();
      m_urgent_message_queue.pop_message(nullptr, 0);
      break;
    case CommMessageType::PropulsionClearCommandQueue:
      m_urgent_message_queue.pop_message(nullptr, 0);
      while (m_message_queue.message_ready()) {
        m_message_queue.pop_message(nullptr, 0);
      }
      break;*/
    case CommMessageType::PropulsionEnableSet: {
      uint8_t enabled;
      m_urgent_message_queue.pop_message((unsigned char*)&enabled, 1);
      m_controller.setEnable(enabled);
      if (!enabled) {
        // Clear queue on disable
        while (m_message_queue.message_ready()) {
          m_message_queue.pop_message(nullptr, 0);
        }
      }
    } break;
    case CommMessageType::PropulsionMotorsEnableSet: {
      uint8_t enabled;
      m_urgent_message_queue.pop_message((unsigned char*)&enabled, 1);
      setMotorsEnable(enabled);
    } break;
    case CommMessageType::PropulsionMotorsVelocitySetpointsSet: {
      float pwm[4];
      m_urgent_message_queue.pop_message((unsigned char*)&pwm, 16);
      setMotorsPwm(pwm[0], pwm[1], true);
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

unsigned char exec_traj_buff[256];  // > 12 for traj params + 16*8 for points = 140

void PropulsionTask::onMsgExecuteTrajectory() {
  int num_points = 0;
  float speed = 0.0;
  float accel = 0.0;
  float deccel = 0.0;
  Vector2D* points = (Vector2D*)(exec_traj_buff + 12);
  auto msg_size = m_message_queue.message_size();

  if (msg_size < 140) {
    m_message_queue.pop_message(exec_traj_buff, 140);
    speed = *(float*)(exec_traj_buff);
    accel = *(float*)(exec_traj_buff + 4);
    deccel = *(float*)(exec_traj_buff + 8);
    points = (Vector2D*)(exec_traj_buff + 12);
    num_points = (msg_size - 12) / sizeof(Vector2D);
  } else {
    m_message_queue.pop_message(NULL, 140);
  }
}

void PropulsionTask::onMsgExecutePointTo() {
  float params[5];
  m_message_queue.pop_message((unsigned char*)&params, sizeof(params));

  m_controller.executePointTo(*(Vector2D*)(params), params[2], params[3], params[4]);
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
    distance = distance + Robot::instance().robotConfig().back_length;
  } else {
    // touched on the front
    distance = distance + Robot::instance().robotConfig().front_length;
  }
  // Project current position on line and adjust yaw
  m_odometry.measureLineNormal(normal, distance);
  pose = m_odometry.pose();
  // Set controller to new pose
  m_controller.resetPose(pose.position.x, pose.position.y, pose.yaw);
}

template <typename T>
void PropulsionTask::ODriveWriteEndpoint(uint16_t endpoint, T val) {
  uint8_t buff[8 + sizeof(T)];

  *reinterpret_cast<uint16_t*>(buff + 0) = m_odrive_seq | 0x4000;
  *reinterpret_cast<uint16_t*>(buff + 2) = endpoint;
  *reinterpret_cast<uint16_t*>(buff + 4) = 0;
  *reinterpret_cast<T*>(buff + 6) = val;
  *reinterpret_cast<uint16_t*>(buff + 6 + sizeof(T)) = c_odrive_key;
  m_odrive_seq = (m_odrive_seq + 1) & 0xbfff;
  Robot::instance().mainExchangeIn().pushMessage(CommMessageType::ODriveRequestPacket, buff,
                                                 sizeof(buff));
}

void PropulsionTask::ODriveSetMotorsEnable(bool enable) {
  // Set velocity control
  ODriveWriteEndpoint(c_odrive_endpoint_control_mode[0], c_odrive_consts_control_mode);
  ODriveWriteEndpoint(c_odrive_endpoint_control_mode[1], c_odrive_consts_control_mode);

  // Enable or disable closed loop control
  uint32_t axis_state = enable ? c_odrive_consts_axis_state[1] : c_odrive_consts_axis_state[0];
  ODriveWriteEndpoint(c_odrive_endpoint_requested_state[0], axis_state);
  ODriveWriteEndpoint(c_odrive_endpoint_requested_state[1], axis_state);
}

void PropulsionTask::ODriveSetVelocitySetPoint(int axis, float vel_setpoint,
                                               float current_feedforward) {
  assert(axis >= 0 && axis < 2);

  ODriveWriteEndpoint(c_odrive_endpoint_input_vel[axis], vel_setpoint);
  ODriveWriteEndpoint(c_odrive_endpoint_input_vel[axis] + 1, current_feedforward);
}

void PropulsionTask::setMotorsEnable(bool enable) {
  if (m_use_simulator) {
    // m_robot_simulator. = left_pwm;
    // m_robot_simulator.m_right_pwm = right_pwm;
  } else if (Robot::instance().robotConfig().use_odrive_uart) {
    ODriveSetMotorsEnable(enable);
  }
}

void PropulsionTask::setMotorsPwm(float left_pwm, float right_pwm, bool immediate) {
  if (m_use_simulator) {
    m_robot_simulator.m_left_pwm = left_pwm;
    m_robot_simulator.m_right_pwm = right_pwm;
  } else if (Robot::instance().robotConfig().use_odrive_uart) {
    if (immediate) {
      ODriveSetVelocitySetPoint(0, -left_pwm, 0);
      ODriveSetVelocitySetPoint(1, right_pwm, 0);
      return;
    }
    if (m_odrive_cnt == 0) {
      ODriveSetVelocitySetPoint(0, left_pwm, 0);
      ODriveSetVelocitySetPoint(1, right_pwm, 0);
    }
    m_odrive_cnt++;
    if (m_odrive_cnt == 10) {
      m_odrive_cnt = 0;
    }
  } else {
    hal::pwm_set(0, left_pwm);
    hal::pwm_set(1, right_pwm);
  }
}

void PropulsionTask::taskFunction() {
  // Register for messages
  // Robot::instance().mainExchangeIn().subscribe({84, 97, &m_message_queue});
  Robot::instance().mainExchangeIn().subscribe({22, 24, &m_urgent_message_queue});
  // Robot::instance().mainExchangeIn().subscribe({80, 83, &m_urgent_message_queue});

  m_use_simulator = Robot::instance().robotConfig().use_simulator;
  m_robot_simulator.m_config = Robot::instance().robotSimulatorConfig();
  // Set task to high
  set_priority(6);

  // Setup odometry
  uint16_t left = hal::encoder_get(0);
  uint16_t right = hal::encoder_get(1);

  m_odometry.reset(left, right);
  m_telemetry_counter = 0;

  while (1) {
    checkStateUpdate();
    if (m_state == Running) {
      doStep();
    }
    // Execute the propulsion control loop every system tick (1ms)
    delay_periodic(1);
  }
}
