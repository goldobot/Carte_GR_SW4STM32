#pragma once

namespace goldobot {
enum class Side : uint8_t
{
	Unknown=0,
	Green=1,
	Orange=2
};

enum class MatchState : uint8_t
{
	Idle, // Initial state
	Debug, // Debug mode
	PreMatch, // Pre match repositioning sequence
	WaitForStartOfMatch, // Ready for match, waiting for start signal
	Match, // Match
	PostMatch // Match finished
};
}
