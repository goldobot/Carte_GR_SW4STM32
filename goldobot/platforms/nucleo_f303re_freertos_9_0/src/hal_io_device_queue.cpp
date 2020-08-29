#include "goldobot/platform/hal_io_device.hpp"

#include "FreeRTOS.h"
#include "task.h"

#include <cstring>


namespace goldobot { namespace platform {


void IODeviceQueue::init(uint8_t* buffer, size_t buffer_size)
{
	m_buffer = buffer;
	m_buffer_end = buffer + buffer_size;
	m_head = buffer;
	m_tail = buffer;
	m_full = 0;
}

size_t IODeviceQueue::size() const
{
	taskDISABLE_INTERRUPTS();
	size_t retval = 0;
	if(m_full) {
		retval = m_buffer_end - m_buffer;
	}
	else if(m_head >= m_tail) {
		retval = m_head - m_tail;
	}
	else {
		retval = (m_buffer_end - m_tail) + (m_head - m_buffer);
	}
	taskENABLE_INTERRUPTS();
	assert(retval <= max_size());
	return retval;
}

size_t IODeviceQueue::max_size() const
{
	return m_buffer_end - m_buffer;
}

size_t IODeviceQueue::space_available() const
{
	return max_size() - size();
}

size_t IODeviceQueue::pop(uint8_t* buffer, size_t buffer_size)
{
	if(buffer_size == 0 || buffer == nullptr)
	{
		return 0;
	}

	uint16_t bytes_to_read = size();
	if(bytes_to_read > buffer_size)
	{
		bytes_to_read = buffer_size;
	}

	if(bytes_to_read == 0)
	{
		return 0;
	}

	uint8_t* tail = m_tail;

	if(tail + bytes_to_read < m_buffer_end)
	{
		std::memcpy(buffer, tail, bytes_to_read);
		tail = tail + bytes_to_read;
	}
	else
	{
		uint16_t size_1 = m_buffer_end - tail;
		uint16_t size_2 = bytes_to_read - size_1;

		std::memcpy(buffer, tail, size_1);
		std::memcpy(buffer + size_1, m_buffer , size_2);
		tail = m_buffer + size_2;
	}

	taskDISABLE_INTERRUPTS();
	m_tail = tail;
	m_full = false;
	taskENABLE_INTERRUPTS();
	return bytes_to_read;
}

size_t IODeviceQueue::push(const uint8_t* buffer, size_t buffer_size)
{
	if(buffer_size == 0 || buffer == nullptr)
	{
		return 0;
	}

	uint16_t bytes_to_write = space_available();
	if(buffer_size < bytes_to_write)
	{
		bytes_to_write = buffer_size;
	}

	uint8_t* head = m_head;
	if(head + bytes_to_write < m_buffer_end)
	{
		std::memcpy(head, buffer, bytes_to_write);
		head = head + bytes_to_write;

	} else
	{
		uint16_t size_1 = m_buffer_end - head;
		uint16_t size_2 = bytes_to_write - size_1;
		std::memcpy(head, buffer, size_1);
		std::memcpy(m_buffer, buffer + size_1, size_2);
		head = m_buffer + size_2;
	}
	taskDISABLE_INTERRUPTS();
	m_head = head;
	if(head == m_tail)
	{
		m_full = true;
	}
	taskENABLE_INTERRUPTS();
	return bytes_to_write;
}

bool IODeviceQueue::init_tx_request(IORequest* request)
{
	// Return false if empty
	if(m_head == m_tail && !m_full)
	{
		return false;
	}

	request->ptr = m_tail;
	if(m_head > m_tail)
	{
		request->size = m_head - m_tail;
	} else
	{
		request->size = m_buffer_end - m_tail;
	}
	return true;
}

void IODeviceQueue::update_tx_request(IORequest* req)
{
	size_t bytes_transmitted = (req->size - req->remaining);
	if(bytes_transmitted == 0)
	{
		return;
	}

	uint8_t* tail = req->ptr + bytes_transmitted;
	if(tail == m_buffer_end)
	{
		tail = m_buffer;
	}

	m_full = false;
	m_tail = tail;
}

bool IODeviceQueue::init_rx_request(IORequest* request)
{
	if(request->state != IORequestState::Ready)
	{
		return false;
	}

	// No space to read data into
	if(m_full)
	{
		request->ptr = 0;
		request->size = 0;
		return false;
	}

	request->ptr = m_head;

	if(m_tail > m_head)
	{
		// Read data up to tail
		request->size = m_tail - m_head;
	}
	else
	{
		// Read data up to end of the buffer
		request->size = m_buffer_end - m_head;
	}
	return true;
}

void IODeviceQueue::update_rx_request(IORequest* req)
{
	if(req->state == IORequestState::Ready)
	{
		return;
	}

	size_t bytes_received = (req->size - req->remaining);
	if(bytes_received == 0)
	{
		return;
	}

	uint8_t* head = req->ptr + bytes_received;
	if(head == m_buffer_end)
	{
		head = m_buffer;
	}

	// If head after reading is equal to tail, the buffer has been filled
	m_full = ((head == m_tail) && req->remaining == 0);
	m_head = head;
	if(m_full)
	{
		int a =1;
	}
}


} }; // namespace goldobot
