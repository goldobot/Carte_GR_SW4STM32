#include "goldobot/robot.hpp"

using namespace goldobot;

Robot Robot::s_instance;

Robot& Robot::instance()
{
	return s_instance;
}

void Robot::init()
{
	m_propulsion_task.init();
	m_comm_task.init();
	m_heartbeat_task.init();
	setOdometryConfig(defaultOdometryConfig());
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

OdometryConfig Robot::defaultOdometryConfig()
{
	OdometryConfig config;
	config.dist_per_count_left = 1.513409e-05;
	config.dist_per_count_right = 1.510209e-05;
	config.wheel_spacing = 3.052931e-01;
	config.encoder_period = 8192;
	config.update_period = 1e-3;
	config.speed_filter_period = 0.005f;
	return config;
}

OdometryConfig Robot::odometryConfig()
{
	return m_odometry_config;
}

void Robot::setOdometryConfig(const OdometryConfig& config)
{
	m_odometry_config = config;
	odometry().setConfig(config);
}
