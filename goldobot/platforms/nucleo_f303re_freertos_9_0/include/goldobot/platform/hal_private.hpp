#pragma once
#include "goldobot/hal.hpp"

#include "FreeRTOS.h"
#include "semphr.h"

#include <assert.h>

namespace goldobot
{

struct IORequest
{
	uint8_t* buffer;
	uint32_t size;
	uint32_t remaining;
};

struct IODeviceQueue
{
	void init(uint8_t* buffer, size_t buffer_size);

	uint16_t size() const;
	uint16_t max_size() const;
	uint16_t space_available() const;

	void push_byte(uint8_t byte) {
		if(m_full) return;
		*m_head = byte;
		auto next_head = m_head + 1;
		m_head = (next_head == m_buffer_end) ? m_buffer : next_head;
		if(m_head == m_tail) m_full = 1;
		assert(m_head < m_buffer_end);
	};

	bool pop_byte(uint8_t* byte) {
		if(m_tail == m_head && !m_full) return false;
		*byte = *m_tail;
		auto next_tail = m_tail + 1;
		m_tail = (next_tail == m_buffer_end) ? 0 : next_tail;
		m_full = 0;
	}

	uint16_t push(const uint8_t* buffer, uint16_t buffer_size);
	uint16_t pop(uint8_t* buffer, uint16_t buffer_size);


	uint8_t* m_buffer;
	uint8_t* m_buffer_end;
	uint8_t* m_head;
	uint8_t* m_tail;
	uint32_t m_full;
};

struct IODevice{
	IODeviceType type;
    void* device_handle;
    IODeviceQueue rx_queue;
    IODeviceQueue tx_queue;
    SemaphoreHandle_t rx_semaphore;
    SemaphoreHandle_t tx_semaphore;
};

} // namespace goldobot
