#include "goldobot/platform/hal_private.hpp"

#include "FreeRTOS.h"
#include "task.h"

#include <cstring>


namespace goldobot
{

void IODeviceQueue::init(uint8_t* buffer, size_t buffer_size)
{
	m_buffer = buffer;
	m_buffer_end = buffer + buffer_size;
	m_head = buffer;
	m_tail = buffer;
	m_full = 0;
}

uint16_t IODeviceQueue::size() const
{
	taskDISABLE_INTERRUPTS();
	uint16_t retval = 0;
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

uint16_t IODeviceQueue::max_size() const
{
	return m_buffer_end - m_buffer;
}

uint16_t IODeviceQueue::space_available() const
{
	return max_size() - size();
}

uint16_t IODeviceQueue::pop(uint8_t* buffer, uint16_t buffer_size)
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

uint16_t IODeviceQueue::push(const uint8_t* buffer, uint16_t buffer_size)
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


} // namespace goldobot
