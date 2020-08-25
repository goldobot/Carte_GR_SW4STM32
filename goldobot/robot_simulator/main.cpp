#include "goldobot/robot.hpp"
#include "robot_emulator.hpp"



void main()
{
	

	RobotEmulator::instance().init();
	goldobot::Hal::init();
	goldobot::Robot::instance().init();

	RobotEmulator::instance().main_loop();
}