#ifndef __GOLDO_COMM_HPP__
#define __GOLDO_COMM_HPP__
#include "goldo_comm/cobs_encoder.hpp"
#include "goldo_comm/cobs_decoder.hpp"
#include "goldo_comm/message_queue.hpp"
#include <chrono>

// proto
//packet header: ff[14 bits] sur 16 bits
// ff: packet type
//01: data packet., followed by sent sequence number and received sequence number, then payload

namespace goldo_comm
{
	namespace chrono =  std::chrono;

	struct iovec {
		void* base;
		size_t len;
	};

	class CommHal
	{
	public:
		virtual ~CommHal();
		virtual uint8_t* lock_read(size_t& buffer_size) = 0;
		virtual void unlock_read(size_t buffer_size) = 0;
		virtual uint8_t* lock_write(size_t& buffer_size) = 0;
		virtual void unlock_write(size_t buffer_size) = 0;
	};

	class LoopbackCommHal : public CommHal
	{
	public:
		~LoopbackCommHal();
		uint8_t* lock_read(size_t& buffer_size) override;
		void unlock_read(size_t buffer_size) override;

		uint8_t* lock_write(size_t& buffer_size) override;
		void unlock_write(size_t buffer_size) override;

	private:
		CircularBuffer<32> m_buffer;
	};
	class Comm
	{
	public:
		bool send(void* buffer, size_t message_size);
		bool send(iovec* vector, size_t count);


		size_t recv(void* buffer, size_t buffer_size);
		size_t recv(iovec* vector, size_t count);
		bool hasMessage();

		void spin(chrono::milliseconds timestamp);

		void setHal(CommHal* hal);

	private:
		MessageQueue<1024, 32> m_send_queue;
		MessageQueue<1024, 32> m_recv_queue;
		CobsEncoder m_encoder;
		CobsDecoder m_decoder;
		CommHal* m_hal_ptr{ nullptr };

		uint8_t m_seq_num{0};
		uint8_t m_ack_num{0};

		bool spinSend();
		bool spinRecv();

		void sendChunk(uint8_t* buffer, size_t size);
		void checkMessage();
	};
}

#endif // __GOLDO_COMMM_HPP__