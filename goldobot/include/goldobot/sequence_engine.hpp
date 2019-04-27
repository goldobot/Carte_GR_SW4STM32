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

	void beginLoad();
	void loadData(unsigned char* data, uint16_t size);
	void endLoad();
	void startSequence(int id);



	SequenceState state() const {return m_state;};

	void finishedMovement() {m_moving = false;};

private:
	SequenceState m_state{SequenceState::WaitForInit};
	Op* m_ops{nullptr};
	uint16_t* m_sequence_offsets{nullptr};
	unsigned char* m_vars{nullptr};
	unsigned char* m_var_types;
	unsigned char m_buffer[4096];
	int m_load_offset{0};
	uint8_t m_num_vars;
	uint8_t m_num_seqs;

	uint16_t m_pc{0};

	bool m_moving{false};

	bool execOp(const Op& op);



	//organisation of buffer:
	// header, 64 bits (uint16_t size, uint16_t crc16, uint8_t num_seq, uint8_tnum_var, reserved)
	// array of 32bits variables, 32bits aligned
	// array of 16bit sequence start offsets
	// array of 32bits ops, 32bits aligned
};

struct SequenceHeader
{
	uint16_t size;
	uint16_t crc16;
	uint8_t num_vars;
	uint8_t num_seqs;
	uint8_t reserved[2];
};
}
