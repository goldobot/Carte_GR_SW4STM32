#pragma once
#include <cstdint>

namespace goldobot
{
	enum class DbgEventType : uint32_t
	{
		DbgEventRobotHoming=0,
		DbgEventStartMatch=1,
		DbgEventStartSequence=2,
		DbgEventExecuteCommand=3,
		DbgEventGoWaypoint=4,
	};

	struct DbgEventStruct
	{
		DbgEventType type;
		uint32_t param1;
		uint32_t param2;
		uint32_t param3;
	};
}
