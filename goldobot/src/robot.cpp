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
}
