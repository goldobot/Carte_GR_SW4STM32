#pragma once
#include <cstdint>
#include "goldobot/message_types.hpp"

namespace goldobot
{

class StrategyEngine
{
public:
	StrategyEngine();


	void onMovementFinished();
	void onSequenceFinished();

private:


};
}
