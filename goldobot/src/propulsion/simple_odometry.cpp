#include <goldobot/propulsion/simple_odometry.hpp>
#include <cmath>

using namespace goldobot;

SimpleOdometry::SimpleOdometry() :
	m_left_encoder(0),
	m_right_encoder(0),
	m_x(0),
	m_y(0),
	m_yaw(0)
{

}

uint16_t SimpleOdometry::leftEncoderValue() const
{
	return m_left_encoder;
}

uint16_t SimpleOdometry::rightEncoderValue() const
{
	return m_right_encoder;
}

const RobotPose& SimpleOdometry::pose() const
{
	return m_pose;
}

void SimpleOdometry::setConfig(const OdometryConfig& config)
{
	m_config = config;
}

void SimpleOdometry::reset(uint16_t left, uint16_t right)
{
	m_left_encoder = left;
	m_right_encoder = right;
}

void SimpleOdometry::update(uint16_t left, uint16_t right)
{
	int diff_left = left - m_left_encoder;
	int diff_right = right - m_right_encoder;

	m_left_encoder = left;
	m_right_encoder = right;

	if(diff_left * 2 > m_config.encoder_period)
	{
		diff_left -= m_config.encoder_period;
	}
	if(diff_left * 2 < -m_config.encoder_period)
	{
		diff_left += m_config.encoder_period;
	}
	if(diff_right * 2 > m_config.encoder_period)
	{
		diff_right -= m_config.encoder_period;
	}
	if(diff_right * 2 < -m_config.encoder_period)
	{
		diff_right += m_config.encoder_period;
	}

	double d_left = diff_left * m_config.dist_per_count_left;
	double d_right = diff_right * m_config.dist_per_count_right;
	double d_yaw = (d_right - d_left) / m_config.wheel_spacing;
	double d_trans = (d_left + d_right) * 0.5;

	m_x += d_trans * cos(m_yaw + d_yaw * 0.5);
	m_y += d_trans * sin(m_yaw + d_yaw * 0.5);
	m_yaw += d_yaw;
	if(m_yaw > M_PI)
	{
		m_yaw -= M_2_PI;
	}
	else if(m_yaw < -M_PI)
	{
		m_yaw += M_2_PI;
	}
	m_pose.position.x = static_cast<float>(m_x);
	m_pose.position.y = static_cast<float>(m_y);
	m_pose.yaw = static_cast<float>(m_yaw);
}

void SimpleOdometry::setPose(const RobotPose& pose)
{
	m_pose = pose;
	m_x = pose.position.x;
	m_y = pose.position.y;
	m_yaw = pose.yaw;
}

