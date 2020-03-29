#ifndef __GOLDO_COMM_COBS_ENCODER_HPP__
#define __GOLDO_COMM_COBS_ENCODER_HPP__
#include <cstdint>
#include <array>
namespace goldo_comm
{
	class CobsEncoder
	{
	public:
		CobsEncoder();

		size_t max_encoded_size(size_t in_size);
		void encode(const uint8_t* in_ptr, size_t& in_size, uint8_t* out_ptr, size_t& out_size);
		void encode_message(const uint8_t* in_ptr, size_t& in_size, uint8_t* out_ptr, size_t& out_size);
		size_t flush(uint8_t* buffer, size_t buffer_size);
		void end_message();

	private:
		using buffer = std::array<uint8_t, 260>;
		using iterator = typename buffer::iterator;
		buffer m_buffer;
		iterator m_read_ptr;
		iterator m_write_ptr;
		iterator m_code_ptr;

		iterator next_ptr(iterator ptr);
		void inc_ptr(iterator& ptr);
		ptrdiff_t ptr_diff(iterator a, iterator b);
	};
}

#endif // __GOLDO_COMM_COBS_ENCODER_HPP__