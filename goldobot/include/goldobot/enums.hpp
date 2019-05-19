#pragma once

namespace goldobot {
enum class Side : uint8_t
{
	Unknown=0,
	Yellow=1,
	Purple=2
};

enum class MatchState : uint8_t
{
	Unconfigured, // Initial state
	Idle, // Initial state after configuration
	PreMatch, // Prematch sequence
	WaitForStartOfMatch, // Ready for match, waiting for start signal
	Match, // Match
	PostMatch, // Match finished
	Debug // Match start switch is disabled
};

enum class ServoType : uint8_t
{
	Unknown=0,
	StandardServo,
	DynamixelAX12,
	DynamixelMX28,
};

enum class ArmState : uint8_t
{
	Unconfigured=0,
	Idle=1,
	Moving=2,
	Blocked=3
};

}
