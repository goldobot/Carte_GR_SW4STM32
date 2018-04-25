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

OdometryConfig Robot::defaultOdometryConfig()
{
	OdometryConfig config;
	config.dist_per_count_left =  1.530456e-05;
	config.dist_per_count_right = 1.510620e-05f;
	config.wheel_spacing = 3.052931e-01;
	config.encoder_period = 8192;
	config.update_period = 1e-3;
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
