#include "goldo_comm/cobs_decoder.hpp"

using namespace goldo_comm;

bool CobsDecoder::decode(const uint8_t* in_ptr, size_t& in_size, uint8_t* out_ptr, size_t& out_size)
{
	const uint8_t* in_beg = in_ptr;
	const uint8_t* in_end = in_ptr + in_size;
	const uint8_t* out_beg = out_ptr;
	const uint8_t* out_end = out_ptr + in_size;

	while (1)
	{
		// Arrived at end of input or output buffer
		if(in_ptr == in_end || out_ptr == out_end)
		{
			in_size = in_ptr - in_beg;
			out_size = out_ptr - out_beg;
			return false;
		}

		// Read until non zero byte received
		if (m_code == 0)
		{
			m_code = *in_ptr++;
			m_counter = m_code;
			continue;
		}

		if (m_counter == 1)
		{
			if (m_code != 255 && *in_ptr != 0)
			{
				*out_ptr++ = 0;
			}
			m_code = *in_ptr++;
			m_counter = m_code;
		}
		else
		{
			*out_ptr++ = *in_ptr++;
			m_counter--;
		}

		if (m_code == 0)
		{
			in_size = in_ptr - in_beg;
			out_size = out_ptr - out_beg;
			return true;
		}
	}
}