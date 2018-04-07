#include "goldobot/tasks/propulsion.hpp"

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

}



void PropulsionTask::taskFunction()
{


	while(1)
	{
		if(m_state == Running)
		{
			doStep();
		}
		// Execute the propulsion control loop every system tick (1ms)
		delayTicks(1);
	}
}
