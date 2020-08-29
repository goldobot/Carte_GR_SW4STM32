#pragma once
#include "goldobot/hal.hpp"

#include "FreeRTOS.h"
#include "semphr.h"

#include <assert.h>

namespace goldobot { namespace platform {

struct IORequest;

typedef void (*IORequestCallback)(IORequest*, void*);
typedef void (*IORequestFunction)(IORequest*, void*);

enum class IORequestState : uint32_t
{
	Ready,
	TxBusy,
	TxComplete,
	RxBusy,
	RxComplete
};

struct IORequest
{
	uint8_t* ptr;
	uint32_t size;
	volatile uint32_t remaining;
	volatile IORequestState state{IORequestState::Ready};
	IORequestCallback callback;
};

struct IODeviceFunctions
{
	IORequestFunction start_rx_request;
	IORequestFunction update_rx_request;
	IORequestFunction abort_rx_request;
	IORequestFunction start_tx_request;
	IORequestFunction update_tx_request;
	IORequestFunction abort_tx_request;
};

struct IODeviceQueue
{
	void init(uint8_t* buffer, size_t buffer_size);

	size_t size() const;
	size_t max_size() const;
	size_t space_available() const;

	size_t push(const uint8_t* buffer, size_t buffer_size);
	size_t pop(uint8_t* buffer, size_t buffer_size);

	// Initialize io request to read data up to the tail or the end of the buffer
	bool init_rx_request(IORequest* request);
	// Update the head according to the count of bytes read by the current request
	void update_rx_request(IORequest* request);

	bool init_tx_request(IORequest* request);
	void update_tx_request(IORequest* request);

	uint8_t* m_buffer;
	uint8_t* m_buffer_end;
	uint8_t* m_head;
	uint8_t* m_tail;
	uint32_t m_full;
};

class IODevice{
public:
	size_t read(uint8_t* buffer, size_t buffer_size);
	size_t write(const uint8_t* buffer, size_t buffer_size);
	void start_rx_fifo();

    void* device_handle;
	IODeviceFunctions* functions;

    IODeviceQueue rx_queue;
    IORequest rx_request;
    SemaphoreHandle_t rx_semaphore;

    IODeviceQueue tx_queue;
    IORequest tx_request;
    SemaphoreHandle_t tx_semaphore;

};

extern IODevice g_io_devices[8];

}}// namespace goldobot::platform
