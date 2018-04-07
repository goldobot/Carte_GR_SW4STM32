#include <goldobot/propulsion/simple_odometry.hpp>

using namespace goldobot;

SimpleOdometry::SimpleOdometry() :
	m_left_encoder(0),
	m_right_encoder(0)
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

}

void SimpleOdometry::reset(uint16_t left, uint16_t right)
{

}

void SimpleOdometry::update(uint16_t left, uint16_t right)
{

}

void SimpleOdometry::setPose(const RobotPose& pose)
{
	m_pose = pose;
}

