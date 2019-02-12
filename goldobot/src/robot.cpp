#include "goldobot/robot.hpp"

using namespace goldobot;

Robot Robot::s_instance;

Robot& Robot::instance()
{
	return s_instance;
}

void Robot::init()
{
	m_robot_config.front_length = 0.170;
	m_robot_config.back_length = 0.135;

	m_comm_task.init();
	m_comm_task.send_message(CommMessageType::Reset,nullptr, 0);
	m_heartbeat_task.init();

	setOdometryConfig(defaultOdometryConfig());
	propulsion().set_config(defaultPropulsionControllerConfig());
	m_propulsion_task.init();
	m_main_task.init();
	m_arms_task.init();
	m_gyro_task.init();

}

void Robot::start()
{
	m_propulsion_task.start();
}

SimpleOdometry& Robot::odometry()
{
	return m_propulsion_task.odometry();
}

PropulsionController& Robot::propulsion()
{
	return m_propulsion_task.controller();
}

UARTCommTask& Robot::comm()
{
	return m_comm_task;
}

MainTask& Robot::mainTask()
{
	return m_main_task;
}

ArmsTask& Robot::arms()
{
	return m_arms_task;
}

FpgaTask& Robot::fpgaTask()
{
	return m_fpga_task;
}

GyroTask& Robot::gyroTask()
{
	return m_gyro_task;
}

OdometryConfig Robot::defaultOdometryConfig()
{
	OdometryConfig config;
	config.dist_per_count_left = 1.513409e-05;
	config.dist_per_count_right = 1.510209e-05;
	config.wheel_spacing = 2.852931e-01;
	config.encoder_period = 8192;
	config.update_period = 1e-3;
	config.speed_filter_period = 0.005f;
	return config;
}

PropulsionControllerConfig Robot::defaultPropulsionControllerConfig()
{
	PropulsionControllerConfig config;

	config.speed_pid_config.period = 1e-3f;
	config.yaw_rate_pid_config.period = 1e-3f;
	config.translation_pid_config.period = 1e-3f;
	config.translation_cruise_pid_config.period = 1e-3f;
	config.yaw_pid_config.period = 1e-3f;

	config.translation_pid_config.kp = 5;
	config.translation_pid_config.ki = 10;
	config.translation_pid_config.lim_iterm = 0.2;

	config.translation_cruise_pid_config.kp = 5;
	config.translation_cruise_pid_config.ki = 15;
	config.translation_cruise_pid_config.lim_iterm = 0.1;

	config.yaw_pid_config.kp = 5;
	config.yaw_pid_config.ki = 20;
	config.yaw_pid_config.lim_iterm = 0.2;


	// Configure speed pid
	config.speed_pid_config.feed_forward = 0.64f;
	config.speed_pid_config.kp = 1.0f;
	config.speed_pid_config.lim_iterm = 0.2;

	// Configure yaw rate pid
	config.yaw_rate_pid_config.feed_forward = 0.12f;
	config.yaw_rate_pid_config.kp = 0.2;

	config.lookahead_distance = 0.15f;
	config.lookahead_time = 0;
	config.static_pwm_limit = 0.3;
	config.moving_pwm_limit = 1;
	config.repositioning_pwm_limit = 0.25;

	return config;
}
OdometryConfig Robot::odometryConfig()
{
	return m_odometry_config;
}

const RobotConfig& Robot::robotConfig() const
{
	return m_robot_config;
}

void Robot::setOdometryConfig(const OdometryConfig& config)
{
	m_odometry_config = config;
	odometry().setConfig(config);
}
