#ifndef __GOLDO_COMM_COBS_DECODER_HPP__
#define __GOLDO_COMM_COBS_DECODER_HPP__
#include <cstdint>
#include <array>
namespace goldo_comm
{
	class CobsDecoder
	{
	public:
		bool decode(const uint8_t* in_ptr, size_t& in_size, uint8_t* out_ptr, size_t& out_size);

	private:
		int m_counter{ 0 };
		int m_code{ 0 };
	};
}

#endif // __GOLDO_COMM_COBS_DECODER_HPP__