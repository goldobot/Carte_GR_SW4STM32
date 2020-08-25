#pragma once
#include <cstdint>
#include <cstddef>

namespace goldobot
{
	class CommSerializer
	{
	public:
		struct Statistics
		{
			uint32_t messages_sent{0};
			uint32_t bytes_sent{0};
			uint32_t buffer_high_watermark{0};
		};
	public:
		bool t_debug{ false };
		CommSerializer(unsigned char* buffer, size_t size);

		size_t size() const;
		size_t availableSize() const;

		bool push_message(uint16_t message_type, const unsigned char* buffer, size_t size);
		size_t pop_data(unsigned char* buffer, size_t size);

		// Return send statistics for the last period and reset them to zero.
		Statistics statistics();

	private:
		void push_data(const unsigned char* buffer, size_t size);

		unsigned char* m_buffer;
		size_t m_buffer_size;
		size_t m_begin_index;
		size_t m_end_index;
		uint8_t m_sequence_number{0};
		bool m_first_message{true};
		Statistics m_statistics;
	};
}
