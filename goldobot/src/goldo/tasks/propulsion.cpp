#include "hal/generic/hal.hpp"
#include "goldo/tasks/propulsion.hpp"
#include "goldo/robot.hpp"

#ifndef M_PI
#define M_PI (3.14159265358979323846)
#endif

using namespace goldobot;

#if 1 /* FIXME : DEBUG */
/* task.h needed for xTaskGetTickCount() */
#include "task.h"
#include "goldo/debug_goldo.hpp"
bool g_dbg_goldo_carac_prop_flag = false;
bool g_dbg_goldo_test_asserv_flag = false;
bool g_dbg_goldo_test_traj_flag = false;
unsigned int g_dbg_goldo_t0;

extern float g_dbg_deriv_filter_alpha;
extern float g_dbg_boost_thr_mot;
extern float g_dbg_zero_thr_mot;
extern int g_dbg_cmd_extra_delay_ms;
typedef struct _goldo_dbg_message {
  unsigned int dbg_param_id;
  union {
    int i;
    float f;
  } dbg_param_val;
} goldo_dbg_message_t;
#endif

PropulsionTask::PropulsionTask():
  m_controller(&m_odometry),
  m_message_queue(m_message_queue_buffer, sizeof(m_message_queue_buffer)),
  m_urgent_message_queue(m_urgent_message_queue_buffer, sizeof(m_urgent_message_queue_buffer))
{
#if 1 /* FIXME : DEBUG */
  g_dbg_goldo_carac_prop_flag = false;
  g_dbg_goldo_test_asserv_flag = false;
  g_dbg_goldo_test_traj_flag = false;
  g_dbg_goldo_t0 = 0;
#endif
}

const char* PropulsionTask::name() const
{
  return "propulsion";
}

void PropulsionTask::doStep()
{
  // Process urgent messages
  while(m_urgent_message_queue.message_ready())
  {
    processUrgentMessage();
  }

  // Process messages
  while(m_message_queue.message_ready() && m_controller.state() == PropulsionController::State::Stopped)
  {
    processMessage();
  }

  while(m_message_queue.message_ready() && m_controller.state() == PropulsionController::State::ManualControl)
  {
    processMessage();
  }

  // adversary detection
  if(Hal::get_gpio(2) && (m_controller.state()==PropulsionController::State::FollowTrajectory) && m_adversary_detection_enabled)
  {
    m_controller.emergencyStop();
  }

  // Update odometry
  uint16_t left;
  uint16_t right;
  Hal::read_encoders(left, right);
  m_odometry.update(left, right);
  m_controller.update();

  while(m_message_queue.message_ready() && m_controller.state() == PropulsionController::State::Stopped)
  {
    processMessage();
  }

  // Check state change
  if(m_controller.state() != m_previous_state)
  {
    uint8_t buff[2];
    buff[0] = (uint8_t)m_controller.state();
    buff[1] = (uint8_t)m_previous_state;
    m_previous_state = m_controller.state();
    Robot::instance().mainExchangeIn().pushMessage(CommMessageType::PropulsionStateChanged,(unsigned char*)buff,2);
    Robot::instance().mainExchangeOut().pushMessage(CommMessageType::PropulsionStateChanged,(unsigned char*)buff,2);
  }
  if(m_controller.state() != PropulsionController::State::Inactive)
  {
    Hal::set_motors_pwm(m_controller.leftMotorPwm(), m_controller.rightMotorPwm());
  }

  // Send periodic telemetry messages
  m_telemetry_counter++;
  if(m_telemetry_counter == 99)
  {
    m_telemetry_counter = 0;
  }

  if((m_telemetry_counter % 20) == 19)
  {
    auto msg = m_controller.getTelemetryEx();
    Robot::instance().mainExchangeOut().pushMessage(
      CommMessageType::PropulsionTelemetryEx,
      (unsigned char*)&msg, sizeof(msg));
  }

  //if((m_telemetry_counter % 5) == 0)
  if((m_telemetry_counter % 20) == 18)
  {
    auto msg = m_controller.getTelemetry();
    Robot::instance().mainExchangeOut().pushMessage(
      CommMessageType::PropulsionTelemetry,
      (unsigned char*)&msg, sizeof(msg));
  }

#if 1 /* FIXME : DEBUG */
  unsigned int curr_t = xTaskGetTickCount();

  if (g_dbg_goldo_carac_prop_flag)
  {
    if(m_telemetry_counter % 10 == 0)
    {
      DbgGoldoVecSimple l_vec_simple;
      Hal::read_encoders(left, right);
      l_vec_simple.clock_ms  = curr_t;
      l_vec_simple.left_odo  = left;
      l_vec_simple.right_odo = right;
      Robot::instance().mainExchangeOut().pushMessage(CommMessageType::DebugGoldoVectSimple,
        (unsigned char*)&l_vec_simple, sizeof(l_vec_simple));
    }

    if ((curr_t-g_dbg_goldo_t0)>1000)
    {
      Hal::set_motors_pwm(0.0, 0.0);
    }

    if ((curr_t-g_dbg_goldo_t0)>2000)
    {
      g_dbg_goldo_carac_prop_flag = false;
    }
  }

  if (g_dbg_goldo_test_asserv_flag)
  {
    if(m_telemetry_counter % 10 == 0)
    {
      DbgGoldoVecAsserv l_vec_asserv;
      RobotPose my_pose = m_odometry.pose();
      RobotPose my_target = m_controller.targetPose();
      double my_x_mm;
      double my_y_mm;
      double my_theta_deg;
      l_vec_asserv.clock_ms              = curr_t - g_dbg_goldo_t0;
      my_x_mm = 1000.0*my_pose.position.x;
      my_y_mm = 1000.0*my_pose.position.y;
      my_theta_deg = my_pose.yaw/M_PI*180.0*1000.0;
      l_vec_asserv.x_mm                  = my_x_mm;
      l_vec_asserv.y_mm                  = my_y_mm;
      l_vec_asserv.theta_deg_1000        = my_theta_deg;
      my_x_mm = 1000.0*my_target.position.x;
      my_y_mm = 1000.0*my_target.position.y;
      my_theta_deg = my_target.yaw/M_PI*180.0*1000.0;
      l_vec_asserv.target_x_mm           = my_x_mm;
      l_vec_asserv.target_y_mm           = my_y_mm;
      l_vec_asserv.target_theta_deg_1000 = my_theta_deg;
      Robot::instance().mainExchangeOut().pushMessage(CommMessageType::DebugGoldoVectAsserv,
        (unsigned char*)&l_vec_asserv, sizeof(l_vec_asserv));
    }

    if ((curr_t-g_dbg_goldo_t0)>8000)
    {
      g_dbg_goldo_test_asserv_flag = false;
    }
  }

  if (g_dbg_goldo_test_traj_flag)
  {
    if(m_telemetry_counter % 100 == 0)
    {
      DbgGoldoVec l_vec;
      double my_x_mm;
      double my_y_mm;
      Vector2D my_target_lookahed = m_controller.targetLookahead();
      my_x_mm = 1000.0*my_target_lookahed.x;
      my_y_mm = 1000.0*my_target_lookahed.y;
      l_vec.clock_ms              = curr_t - g_dbg_goldo_t0;
      l_vec.x_mm                  = my_x_mm;
      l_vec.y_mm                  = my_y_mm;
      l_vec.theta_deg_1000        = 0;
      l_vec.left_odo              = 0;
      l_vec.right_odo             = 0;
      Robot::instance().mainExchangeOut().pushMessage(CommMessageType::DebugGoldoVectTraj,
        (unsigned char*)&l_vec, sizeof(l_vec));
    }

    if ((curr_t-g_dbg_goldo_t0)>30000)
    {
      g_dbg_goldo_test_traj_flag = false;
    }
  }

#endif

}

void PropulsionTask::processMessage()
{
  auto message_type = (CommMessageType)m_message_queue.message_type();

#if 1 /* FIXME : DEBUG */
  if ((message_type==CommMessageType::DbgPropulsionExecuteRotation) ||
      (message_type==CommMessageType::DbgPropulsionExecuteTranslation))
  {
    g_dbg_goldo_test_asserv_flag = true;
    g_dbg_goldo_t0  = xTaskGetTickCount();
  }
  if ((message_type==CommMessageType::DbgPropulsionExecuteTrajectory))
  {
    g_dbg_goldo_test_traj_flag = true;
    g_dbg_goldo_t0  = xTaskGetTickCount();
  }
#endif

  switch(message_type)
  {
  case CommMessageType::PropulsionExecuteTrajectory:
  case CommMessageType::DbgPropulsionExecuteTrajectory:
    onMsgExecuteTrajectory();
    break;
  case CommMessageType::PropulsionExecuteRotation:
  case CommMessageType::DbgPropulsionExecuteRotation:
    {
      float params[4];
      m_message_queue.pop_message((unsigned char*)&params, sizeof(params));
      m_controller.executeRotation(params[0], params[1], params[2], params[3]);
    }
    break;
  case CommMessageType::PropulsionExecuteTranslation:
  case CommMessageType::DbgPropulsionExecuteTranslation:
    {
      float params[4];
      m_message_queue.pop_message((unsigned char*)&params, sizeof(params));
      m_controller.executeTranslation(params[0], params[1], params[2], params[3]);
    }
    break;
  case CommMessageType::PropulsionExecutePointTo:
    onMsgExecutePointTo();
    break;
  case CommMessageType::PropulsionExecuteFaceDirection:
    {
      float params[4];
      m_message_queue.pop_message((unsigned char*)&params, sizeof(params));
      m_controller.executeFaceDirection(params[0], params[1], params[2], params[3]);
    }
    break;
  case CommMessageType::PropulsionExecuteMoveTo:
    {
      float params[5];
      m_message_queue.pop_message((unsigned char*)&params, sizeof(params));
      m_controller.executeMoveTo(*(Vector2D*)(params), params[2], params[3], params[4]);
    }
    break;
  case CommMessageType::PropulsionExecuteReposition:
    {
      float params[2];
      m_message_queue.pop_message((unsigned char*)&params, sizeof(params));
      m_controller.executeRepositioning(params[0], params[1]);
    }
    break;
  case CommMessageType::PropulsionEnterManualControl:
    m_controller.enterManualControl();
    break;
  case CommMessageType::PropulsionExitManualControl:
    m_controller.exitManualControl();
    break;
  case CommMessageType::PropulsionSetControlLevels:
    {
      uint8_t buff[2];
      m_message_queue.pop_message((unsigned char*)buff,2);
      m_controller.setControlLevels(buff[0], buff[1]);
    }
    break;
  case CommMessageType::PropulsionSetTargetPose:
    {
      RobotPose pose;
      m_message_queue.pop_message((unsigned char*)&pose, sizeof(pose));
      m_controller.setTargetPose(pose);
    }
    break;
  case CommMessageType::PropulsionMeasureNormal:
    {
      float buff[2];
      m_message_queue.pop_message((unsigned char*)&buff, sizeof(buff));
      measureNormal(buff[0], buff[1]);
    }
    break;
#if 1 /* FIXME : DEBUG */
  case CommMessageType::CmdEmergencyStop:
    m_controller.emergencyStop();
    m_message_queue.pop_message(nullptr, 0);
    break;
  case CommMessageType::PropulsionClearError:
    m_controller.clearError();
    m_message_queue.pop_message(nullptr, 0);
    break;
  case CommMessageType::DebugGoldoSetParam:
    {
      goldo_dbg_message_t dbg_msg;
      bool print_only = false;
      bool is_float = false;
      char *pname = "UNKNOWN";
      int old_i = 0;
      int new_i = 0;
      float old_f = 0.0f;
      float new_f = 0.0f;

      m_message_queue.pop_message((unsigned char*)&dbg_msg, sizeof(dbg_msg));

      if ((dbg_msg.dbg_param_id&=0x80000000)!=0) print_only = true;
      switch (dbg_msg.dbg_param_id)
      {
      case 1:
        pname = "g_dbg_boost_thr_mot";
        old_f = g_dbg_boost_thr_mot;
        new_f = dbg_msg.dbg_param_val.f;
        if (!print_only) g_dbg_boost_thr_mot = new_f;
        is_float = true;
        break;
      case 2:
        pname = "g_dbg_zero_thr_mot";
        old_f = g_dbg_zero_thr_mot;
        new_f = dbg_msg.dbg_param_val.f;
        if (!print_only) g_dbg_zero_thr_mot = new_f;
        is_float = true;
        break;
      case 3:
        pname = "g_dbg_cmd_extra_delay_ms";
        old_i = g_dbg_cmd_extra_delay_ms;
        new_i = dbg_msg.dbg_param_val.i;
        if (!print_only) g_dbg_cmd_extra_delay_ms = new_i;
        is_float = false;
        break;
      case 4:
        pname = "g_dbg_deriv_filter_alpha";
        old_f = g_dbg_deriv_filter_alpha;
        new_f = dbg_msg.dbg_param_val.f;
        if (!print_only) g_dbg_deriv_filter_alpha = new_f;
        is_float = true;
        break;
      default:
        break;
      }

      if (print_only)
      {
        new_f = old_f;
        new_i = old_i;
      }

      if (is_float)
      {
        goldo_send_log("DBG PARAM %s: %f -> %f", pname, old_f, new_f);
      }
      else
      {
        goldo_send_log("DBG PARAM %s: %d -> %d", pname, old_i, new_i);
      }
    }

    break;
#endif
  default:
    m_message_queue.pop_message(nullptr, 0);
    break;
  }
}

void PropulsionTask::processUrgentMessage()
{
  auto message_type = (CommMessageType)m_urgent_message_queue.message_type();

  switch(message_type)
  {
  case CommMessageType::PropulsionSetPose:
    {
      float pose[3];
      m_urgent_message_queue.pop_message((unsigned char*)&pose, 12);
      m_controller.resetPose(pose[0], pose[1], pose[2]);
    }
    break;
  case CommMessageType::PropulsionSetAdversaryDetectionEnable:
    {
      uint8_t buff;
      m_urgent_message_queue.pop_message((unsigned char*)&buff,1);
      m_adversary_detection_enabled = (bool)buff;
    }
    break;
  case CommMessageType::DbgGetOdometryConfig:
    {
      auto config = m_odometry.config();
      Robot::instance().mainExchangeOut().pushMessage(
        CommMessageType::DbgGetOdometryConfig,
        (unsigned char*)&config, sizeof(config));
      m_urgent_message_queue.pop_message(nullptr, 0);
    }
    break;
  case CommMessageType::DbgSetOdometryConfig:
    {
      OdometryConfig config;
      m_urgent_message_queue.pop_message((unsigned char*)&config, sizeof(config));
      m_odometry.setConfig(config);
    }
    break;
  case CommMessageType::DbgGetPropulsionConfig:
    {
      auto config = m_controller.config();
      Robot::instance().mainExchangeOut().pushMessage(
        CommMessageType::DbgGetPropulsionConfig,
        (unsigned char*)&config, sizeof(config));
      m_urgent_message_queue.pop_message(nullptr, 0);
    }
    break;
  case CommMessageType::DbgSetPropulsionConfig:
    {
      PropulsionControllerConfig config;
      m_urgent_message_queue.pop_message((unsigned char*)&config, sizeof(config));
      m_controller.setConfig(config);
    }
    break;
  case CommMessageType::CmdEmergencyStop:
    m_controller.emergencyStop();
    m_urgent_message_queue.pop_message(nullptr, 0);
    break;
  case CommMessageType::PropulsionClearError:
    m_controller.clearError();
    m_urgent_message_queue.pop_message(nullptr, 0);
    break;
  case CommMessageType::PropulsionClearCommandQueue:
    m_urgent_message_queue.pop_message(nullptr, 0);
    while(m_message_queue.message_ready())
    {
      m_message_queue.pop_message(nullptr, 0);
    }
    break;
  case CommMessageType::PropulsionSetEnable:
    {
      uint8_t enabled;
      m_urgent_message_queue.pop_message((unsigned char*)&enabled, 1);
      m_controller.setEnable(enabled);
      if(!enabled)
      {
        // Clear queue on disable
        while(m_message_queue.message_ready())
        {
          m_message_queue.pop_message(nullptr, 0);
        }
      }
    }
    break;
  case CommMessageType::SetMotorsEnable:
    {
      uint8_t enabled;
      m_urgent_message_queue.pop_message((unsigned char*)&enabled, 1);
      Hal::set_motors_enable(enabled);
    }
    break;
  case CommMessageType::SetMotorsPwm:
    {
      float pwm[2];
      m_urgent_message_queue.pop_message((unsigned char*)&pwm, 8);
      Hal::set_motors_pwm(pwm[0], pwm[1]);
    }
    break;
  case CommMessageType::PropulsionMeasurePoint:
    {
      float buff[4];
      m_urgent_message_queue.pop_message((unsigned char*)&buff, sizeof(buff));
      m_odometry.measurePerpendicularPoint(buff[0], buff[1], *(Vector2D*)(buff+2));
      auto pose = m_odometry.pose();
      // Set controller to new pose
      m_controller.resetPose(pose.position.x, pose.position.y, pose.yaw);
    }
    break;
#if 1 /* FIXME : DEBUG */
  case CommMessageType::DbgSetMotorsPwm:
    {
      float pwm[2];
      m_urgent_message_queue.pop_message((unsigned char*)&pwm, 8);
      uint32_t my_ts = xTaskGetTickCount();
      Hal::set_motors_pwm(pwm[0], pwm[1]);
      if ((fabs(pwm[0])>0.01) || (fabs(pwm[1])>0.01))
      {
        DbgGoldoVecSimple _dbg_goldo_vec;
        uint16_t left;
        uint16_t right;
        Hal::read_encoders(left, right);

        _dbg_goldo_vec.clock_ms = my_ts;
        _dbg_goldo_vec.left_odo = left;
        _dbg_goldo_vec.right_odo = right;

        Robot::instance().mainExchangeOut().pushMessage(CommMessageType::DebugGoldoVectSimple,
          (unsigned char*)&_dbg_goldo_vec, sizeof(_dbg_goldo_vec));
        g_dbg_goldo_carac_prop_flag = true;
        g_dbg_goldo_t0  = xTaskGetTickCount();
      }
      else
      {
        g_dbg_goldo_carac_prop_flag = false;
      }
    }
    break;
#endif
  default:
    m_urgent_message_queue.pop_message(nullptr, 0);
    break;
  }
}

#if 1 /* FIXME : DEBUG : GOLDO */
extern bool debug_traj_flag;
extern int debug_num_points;
extern short debug_traj_x_mm[];
extern short debug_traj_y_mm[];
#endif

unsigned char exec_traj_buff[256];// > 12 for traj params + 16*8 for points = 140

void PropulsionTask::onMsgExecuteTrajectory()
{
  int num_points = 0;
  float speed = 0.0;
  float accel = 0.0;
  float deccel = 0.0;
  Vector2D* points = (Vector2D*)(exec_traj_buff+12);
  auto msg_size = m_message_queue.message_size();

  if (msg_size<140)
  {
    m_message_queue.pop_message(exec_traj_buff,140);
    speed = *(float*)(exec_traj_buff);
    accel = *(float*)(exec_traj_buff+4);
    deccel = *(float*)(exec_traj_buff+8);
    points = (Vector2D*)(exec_traj_buff+12);
    num_points = (msg_size-12)/sizeof(Vector2D);
  }
  else 
  {
    m_message_queue.pop_message(NULL,140);
  }

#if 1 /* FIXME : DEBUG : GOLDO */
  /*if (!debug_traj_flag)*/
  {
    debug_traj_flag = false;
    if (num_points>0)
    {
      int i;
      double tmp_f;
      debug_num_points = num_points;
      for (i=0; i<num_points; i++) {
        tmp_f = 1000.0*points[i].x;
        debug_traj_x_mm[i] = tmp_f;
        tmp_f = 1000.0*points[i].y;
        debug_traj_y_mm[i] = tmp_f;
      }
    }
    else
    {
      debug_num_points = 1;
      debug_traj_x_mm[0] = 0x7fff;
      debug_traj_y_mm[0] = 0x7fff;
    }
    debug_traj_flag = true;
  }
#endif

#if 1 /* FIXME : DEBUG : GOLDO */
  if (num_points>0)
    m_controller.executeTrajectory(points, num_points, speed, accel, deccel);
#endif
}

void PropulsionTask::onMsgExecutePointTo()
{
  float params[5];
  m_message_queue.pop_message((unsigned char*)&params, sizeof(params));

#if 1 /* FIXME : DEBUG : GOLDO */
  /*if (!debug_traj_flag)*/
  {
    debug_traj_flag = false;
    {
      int i;
      double tmp_f;
      debug_num_points = 1;
      tmp_f = 1000.0*params[0];
      debug_traj_x_mm[0] = tmp_f;
      tmp_f = 1000.0*params[1];
      debug_traj_y_mm[0] = tmp_f;
    }
    debug_traj_flag = true;
  }
#endif

  m_controller.executePointTo(*(Vector2D*)(params), params[2], params[3], params[4]);
}

SimpleOdometry& PropulsionTask::odometry()
{
  return m_odometry;
}

PropulsionController& PropulsionTask::controller()
{
  return m_controller;
}

void PropulsionTask::measureNormal(float angle, float distance)
{
  auto pose = m_odometry.pose();
  Vector2D normal{cos(angle), sin(angle)};
  // Check if front or back is touching the border
  float dot = normal.x * cos(pose.yaw) + normal.y * sin(pose.yaw);
  if(dot >0)
  {
    // border normal is aligned with robot yaw
    // means the robot back is touching the border
    distance = distance + Robot::instance().robotConfig().back_length;
  } 
  else
  {
    // touched on the front
    distance = distance + Robot::instance().robotConfig().front_length;
  }
  // Project current position on line and adjust yaw
  m_odometry.measureLineNormal(normal, distance);
  pose = m_odometry.pose();
  // Set controller to new pose
  m_controller.resetPose(pose.position.x, pose.position.y, pose.yaw);
}

void PropulsionTask::taskFunction()
{
  // Register for messages
  Robot::instance().mainExchangeIn().subscribe({84,97, &m_message_queue});
  Robot::instance().mainExchangeIn().subscribe({64,68, &m_urgent_message_queue});
  Robot::instance().mainExchangeIn().subscribe({80,83, &m_urgent_message_queue});
  Robot::instance().mainExchangeIn().subscribe({32,32, &m_urgent_message_queue});
  Robot::instance().mainExchangeIn().subscribe({98,102, &m_urgent_message_queue});
  Robot::instance().mainExchangeIn().subscribe({103,126, &m_message_queue});
#if 1 /* FIXME : DEBUG */
  Robot::instance().mainExchangeIn().subscribe({32,32, &m_message_queue});
  Robot::instance().mainExchangeIn().subscribe({99,100, &m_message_queue});
#endif

  // Set task to high
  set_priority(6);

  // Setup odometry
  uint16_t left;
  uint16_t right;
  Hal::read_encoders(left, right);
  m_odometry.reset(left, right);
  m_telemetry_counter = 0;

  while(1)
  {
    checkStateUpdate();
    if(m_state == Running)
    {
      doStep();
    }

    // Execute the propulsion control loop every system tick (1ms)
    delay_periodic(1);
  }

}
