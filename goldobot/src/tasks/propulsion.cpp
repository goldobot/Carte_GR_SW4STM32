#include "goldobot/tasks/propulsion.hpp"
#include "goldobot/hal.hpp"

using namespace goldobot;

PropulsionTask::PropulsionTask()
{
}

const char* PropulsionTask::name() const
{
	return "propulsion";
}


void PropulsionTask::doStep()
{
	// Update odometry
	uint16_t left;
	uint16_t right;
	Hal::read_encoders(left, right);
	m_odometry.update(left, right);

}

SimpleOdometry& PropulsionTask::odometry()
{
	return m_odometry;
}



void PropulsionTask::taskFunction()
{
	// Setup odometry
	uint16_t left;
	uint16_t right;
	Hal::read_encoders(left, right);
	m_odometry.reset(left, right);

	while(1)
	{
		checkStateUpdate();
		if(m_state == Running)
		{
			doStep();
		}
		// Execute the propulsion control loop every system tick (1ms)
		delayTicks(1);
	}
}
