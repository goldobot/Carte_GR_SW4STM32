#include "goldo_comm/comm.hpp"
#include "goldo_comm/crc32c.hpp"

#include <cassert>

using namespace goldo_comm;

void Comm::setHal(CommHal* hal) { m_hal_ptr = hal; }

bool Comm::hasMessage()
{
    return !m_recv_queue.empty();
}


bool Comm::send(void* buffer, size_t message_size) {
  iovec vec{buffer, message_size};
  return send(&vec, 1);
}

bool Comm::send(iovec* vector, size_t count) {
  size_t total_size = 0;
  for (int i = 0; i < count; i++) {
    total_size += vector[i].len;
  }
  //2 bytes header plus 4 bytes crc32
  if (total_size == 0 || total_size + 6 > m_send_queue.available_message_size()) {
    return false;
  }

  uint8_t header[2] = { static_cast<uint8_t>((m_seq_num & 0x7f) >> 1), ((m_seq_num << 7)  & 0xff) || (m_ack_num & 0x7f) };
  uint32_t checksum = crc32c(0xffffffff, (uint8_t*)header, 2);
  sendChunk((uint8_t*)&header, 2);
  for (int i = 0; i < count; i++)
  {
	  sendChunk((uint8_t*)vector[i].base, vector[i].len);
      checksum = crc32c(checksum, (uint8_t*)vector[i].base, vector[i].len);
  }
  sendChunk((uint8_t*)&checksum, 4);
  m_send_queue.end_message();
  m_seq_num = (m_seq_num + 1) & 0x7f;
  return true;
}

void Comm::sendChunk(uint8_t* buffer, size_t size)
{
	uint8_t* ptr = (uint8_t*)buffer;
	while (ptr - buffer < size)
	{
		size_t chunk_size = size - (ptr - buffer);
		uint8_t* out_ptr = m_send_queue.lock_write(chunk_size);
		memcpy(out_ptr, ptr, chunk_size);
		m_send_queue.unlock_write(chunk_size);
		ptr += chunk_size;
	}
}

size_t Comm::recv(void* buffer, size_t buffer_size) {
  iovec vec{buffer, buffer_size};
  return recv(&vec, 1);
}

size_t Comm::recv(iovec* vector, size_t count) {
    if(m_recv_queue.empty())
    {
        return 0;
    }
    size_t remaining_size = m_recv_queue.message_size() - 6;
    m_recv_queue.unlock_read(2);
    size_t total_len = 0;
    for (int i = 0; i < count; i++)
    {
        uint8_t* ptr_out = (uint8_t*)vector[i].base;
        size_t remaining_len = std::min(vector[i].len, remaining_size);
        while (remaining_len > 0)
        {
            size_t chunk_size = remaining_len;
            const uint8_t* ptr = m_recv_queue.lock_read(chunk_size);
            if(chunk_size == 0)
            {
                m_recv_queue.unlock_read(4);
                m_recv_queue.pop_message();
                return total_len;
            }
            memcpy(ptr_out, ptr, chunk_size);
            m_recv_queue.unlock_read(chunk_size);
            ptr_out += chunk_size;
            remaining_len -= chunk_size;
            remaining_size -= chunk_size;
            total_len += chunk_size;
        }        
    }
    m_recv_queue.pop_message();
    return total_len;
}

void Comm::spin(chrono::milliseconds timestamp) {
  while (!m_send_queue.empty() && spinSend()) {
  }
  while(spinRecv())
  { }
}
bool Comm::spinSend() {
  while (m_send_queue.message_size()) {
    size_t buffer_size = m_send_queue.message_size();
	size_t out_size = buffer_size;
    const uint8_t* ptr = m_send_queue.lock_read(buffer_size);
    uint8_t* out_ptr = m_hal_ptr->lock_write(out_size);

	m_encoder.encode(ptr, buffer_size, out_ptr, out_size);
    m_send_queue.unlock_read(buffer_size);
    m_hal_ptr->unlock_write(out_size);
    if (buffer_size == 0) {
      return false;
    }
  }
  m_send_queue.pop_message();
  m_encoder.end_message();

  while (1)
  {
	  size_t out_size = 260;
	  uint8_t* out_ptr = m_hal_ptr->lock_write(out_size);
	  out_size = m_encoder.flush(out_ptr, out_size);
	  m_hal_ptr->unlock_write(out_size);
	  if (out_size == 0)
	  {
		  return true;
	  }
  }
}

bool Comm::spinRecv() {
  if (m_recv_queue.full())
  {
      return false;
  }
  size_t buffer_size = m_recv_queue.available_message_size();
  const uint8_t* ptr = m_hal_ptr->lock_read(buffer_size);
  uint8_t* out_ptr = m_recv_queue.lock_write(buffer_size);
  if (buffer_size == 0)
  {
      return false;
  }

  size_t in_size = buffer_size;
  size_t out_size = buffer_size;

  bool is_msg_end = m_decoder.decode(ptr, in_size, out_ptr, out_size);;

  m_hal_ptr->unlock_read(in_size);
  m_recv_queue.unlock_write(out_size);
  if (is_msg_end)
  {      
      checkMessage();
  }

  return true;
}

void Comm::checkMessage()
{
    // Check message crc
	size_t msg_size = 0;
	uint32_t checksum = -1;

	if (m_recv_queue.m_buffer.m_write_ptr >= m_recv_queue.m_msg_idx[m_recv_queue.m_write_idx])
	{
		size_t msg_size = m_recv_queue.m_buffer.m_write_ptr - m_recv_queue.m_msg_idx[m_recv_queue.m_write_idx];
		checksum = crc32c(0xffffffff, &*m_recv_queue.m_msg_idx[m_recv_queue.m_write_idx], msg_size);
	}
	else
	{
		size_t s1 = m_recv_queue.m_buffer.m_buffer.end() - m_recv_queue.m_msg_idx[m_recv_queue.m_write_idx];
		size_t s2 = m_recv_queue.m_buffer.m_write_ptr - m_recv_queue.m_buffer.m_buffer.begin();
		checksum = crc32c(0xffffffff, &*m_recv_queue.m_msg_idx[m_recv_queue.m_write_idx], s1);
		checksum = crc32c(checksum, &*m_recv_queue.m_buffer.m_buffer.begin(), s2);
	}
    m_recv_queue.end_message();
}

CommHal::~CommHal() {}
LoopbackCommHal::~LoopbackCommHal() {}

uint8_t* LoopbackCommHal::lock_read(size_t& buffer_size) {
  return m_buffer.lock_read(buffer_size);
}

void LoopbackCommHal::unlock_read(size_t buffer_size) {
  m_buffer.unlock_read(buffer_size);
}

uint8_t* LoopbackCommHal::lock_write(size_t& buffer_size) {
  return m_buffer.lock_write(buffer_size);
}

void LoopbackCommHal::unlock_write(size_t buffer_size) {
  m_buffer.unlock_write(buffer_size);
}
