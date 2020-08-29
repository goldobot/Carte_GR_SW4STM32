#include "goldobot/platform/hal_io_device.hpp"

namespace goldobot { namespace platform {

IODevice g_io_devices[8];

void io_device_tx_complete_callback_fifo(IORequest* req, void* user_ptr)
{
	assert(req->state == IORequestState::TxComplete);
	auto device = static_cast<IODevice*>(user_ptr);
	device->tx_queue.update_tx_request(req);
	req->state = goldobot::platform::IORequestState::Ready;
	if(device->tx_queue.init_tx_request(req))
	{
		device->functions->start_tx_request(req, device->device_handle);
	}
}

void io_device_rx_complete_callback_fifo(IORequest* req, void* user_ptr)
{
	assert(req->state == IORequestState::RxComplete);
	auto device = static_cast<IODevice*>(user_ptr);
	device->rx_queue.update_rx_request(req);
	req->state = goldobot::platform::IORequestState::Ready;
	if(device->rx_queue.init_rx_request(req))
	{
		device->functions->start_rx_request(req, device->device_handle);
	}
}

void IODevice::start_rx_fifo()
{
	rx_queue.init_rx_request(&rx_request);
	rx_request.callback = &io_device_rx_complete_callback_fifo;
	functions->start_rx_request(&rx_request, device_handle);
}

size_t IODevice::read(uint8_t* buffer, size_t buffer_size)
{
	if(buffer_size == 0)
	{
		return 0;
	}

	if(true) // non blocking io
	{
		// Update state of rx request to read received data without waiting for end of current transfer.
		taskDISABLE_INTERRUPTS();
		functions->update_rx_request(&rx_request, device_handle);
		rx_queue.update_rx_request(&rx_request);
		taskENABLE_INTERRUPTS();

		auto bytes_read = rx_queue.pop(buffer, buffer_size);

		if(rx_request.state == IORequestState::Ready)
		{
			if(rx_queue.init_rx_request(&rx_request))
			{
				rx_request.callback = &io_device_rx_complete_callback_fifo;
				functions->start_rx_request(&rx_request, device_handle);
			}
		}
		return bytes_read;
	}
	return 0;
}

size_t IODevice::write(const uint8_t* buffer, size_t buffer_size)
{
	if(buffer_size == 0)
	{
		return 0;
	}

	if(true) // non blocking io
	{
		// Update state of tx request to get an accurate measure of space remaining in the tx queue.
		taskDISABLE_INTERRUPTS();
		functions->update_tx_request(&tx_request, device_handle);
		tx_queue.update_tx_request(&tx_request);
		taskENABLE_INTERRUPTS();

		uint16_t bytes_written = tx_queue.push(buffer, buffer_size);

		// If not currently transmitting, start tx request
		if(tx_request.state == IORequestState::Ready)
		{
			if(tx_queue.init_tx_request(&tx_request))
			{
				tx_request.callback = &io_device_tx_complete_callback_fifo;
				functions->start_tx_request(&tx_request, device_handle);
			}
		}
		return bytes_written;
	}
	return 0;
}

}}// namespace goldobot::platform
