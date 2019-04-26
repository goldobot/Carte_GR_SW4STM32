#pragma once
#include <cstdint>


namespace goldobot
{
struct Op
{
	uint8_t opcode;
	uint8_t arg1;
	uint8_t arg2;
	uint8_t arg3;
};

enum class SequenceState : uint8_t
{
	WaitForInit,
	Idle,
	Executing
};

class SequenceEngine
{
public:
	SequenceEngine();

	void doStep(); /*< Execute instructions until forced to wait*/

	void init();
	void deinit();

	void loadBuffer(unsigned char* data, uint16_t size, uint16_t offset);

private:
	SequenceState m_state{SequenceState::WaitForInit};
	Op* m_ops;
	uint16_t* m_sequence_offsets;
	unsigned char* m_vars;
	unsigned char* m_var_types;
	unsigned char m_buffer[4096];



	//organisation of buffer:
	// header, 64 bits (uint16_t size, uint16_t crc16, uint8_t num_seq, num_var, reserved)
	// array of 4bits variable types
	// array of 32bits variables, 32bits aligned
	// array of 16bit sequence start offsets
	// array of 32bits ops, 32bits aligned
};
}
