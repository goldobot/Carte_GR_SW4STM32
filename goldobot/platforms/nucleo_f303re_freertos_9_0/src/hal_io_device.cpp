#include "goldobot/platform/hal_io_device.hpp"

#include "goldobot/platform/hal_i2c.hpp"
#include "goldobot/platform/hal_spi.hpp"
#include "goldobot/platform/hal_uart.hpp"

namespace goldobot {
namespace hal {
namespace platform {

IODevice g_io_devices[8];

void init_io_device(IODeviceConfig* config) {
  IODevice* device = &g_io_devices[config->io_device_id];
  device->io_flags = config->io_flags;

  if (config->rx_buffer_size > 0) {
    uint8_t* rx_buffer = static_cast<uint8_t*>(pvPortMalloc(config->rx_buffer_size));
    device->rx_queue.init(rx_buffer, config->rx_buffer_size);
  }

  if (config->rx_buffer_size > 0) {
    uint8_t* tx_buffer = static_cast<uint8_t*>(pvPortMalloc(config->rx_buffer_size));
    device->tx_queue.init(tx_buffer, config->tx_buffer_size);
  }

  device->req_finished_semaphore = xSemaphoreCreateBinary();

  switch (config->device_type) {
    case DeviceType::Uart:
      hal_usart_init(device, static_cast<const IODeviceConfigUart*>(config));
      break;
    case DeviceType::I2c:
      hal_i2c_init(device, static_cast<const IODeviceConfigI2c*>(config));
      break;
    case DeviceType::Spi:
      hal_spi_init(device, static_cast<const IODeviceConfigSpi*>(config));
      break;
    default:
      break;
  }

  // Non blocking io (fifo mode)
  if (!(device->io_flags & IODeviceFlags::RxBlocking) && device->rx_functions != nullptr) {
    device->try_start_rx_fifo();
  }
}

bool io_device_rx_complete_callback_fifo(IORequest* req, IORequestStatus status) {
  auto device = reinterpret_cast<IODevice*>(req->userdata);
  device->rx_queue.unmap_push(req->rx_ptr, status.size);

  uint8_t* ptr;
  auto size = device->rx_queue.map_push(&ptr);

  if (size > 0 && status.code == IORequestStatus::Success) {
     req->rx_ptr = ptr;
	 req->size = size;
    return true;
  } else
  {
	  device->rx_request.state = IORequestState::Ready;
	  return false;
  }
}

bool io_device_tx_complete_callback_fifo(IORequest* req, IORequestStatus status) {
  auto device = reinterpret_cast<IODevice*>(req->userdata);
  device->tx_queue.unmap_pop(req->tx_ptr, status.size);

  uint8_t* ptr;
  auto size = device->tx_queue.map_pop(&ptr);

  if (size > 0) {
    req->tx_ptr = ptr;
    req->size = size;
    return true;
  } else
  {
	  device->tx_request.state = IORequestState::Ready;
	  return false;
  }
}

void IODevice::try_start_rx_fifo() {
  auto rx_state = rx_request.state.load();
  if (rx_state == IORequestState::Ready) {
	  uint8_t* ptr;
	  auto size = rx_queue.map_push(&ptr);

	  if (size > 0) {
		auto req = &rx_request;
		req->rx_ptr = ptr;
		req->size = size;
		req->callback = &io_device_rx_complete_callback_fifo;
		req->userdata = this;
		rx_functions->start_request(&rx_request, device_index);
	  }
	};
  }

void IODevice::start_tx_fifo() {
  uint8_t* ptr;
  auto size = tx_queue.map_pop(&ptr);

  if (size > 0) {
	auto req = &tx_request;
    tx_request.tx_ptr = ptr;
    tx_request.size = size;
    tx_request.callback = &io_device_tx_complete_callback_fifo;
    req->userdata = this;
    tx_functions->start_request(&tx_request, device_index);
  }
}

bool io_device_rx_complete_callback_blocking(IORequest* req, IORequestStatus status) {
  return false;
}

bool io_device_tx_complete_callback_blocking(IORequest* req, IORequestStatus status) {
  return false;
}


void IODevice::execute(IORequest req, uint32_t timeout) {

	if(req.rx_ptr != nullptr)
	{
		assert(rx_request.state != IORequestState::Busy);
		rx_request.state = IORequestState::Ready;
		rx_request.rx_ptr = req.rx_ptr;
		rx_request.tx_ptr = nullptr;
		rx_request.size = req.size;
		rx_request.callback = req.callback;
		rx_request.userdata = req.userdata;
		rx_functions->start_request(&rx_request, device_index);
	} else
	{
		assert(tx_request.state != IORequestState::Busy);
		tx_request.state = IORequestState::Ready;
		tx_request.rx_ptr = nullptr;
		tx_request.tx_ptr = req.tx_ptr;
		tx_request.size = req.size;
		tx_request.callback = req.callback;
		tx_request.userdata = req.userdata;
		tx_functions->start_request(&tx_request, device_index);
	}
    auto tick_count_timeout = xTaskGetTickCount() + timeout;
    while (true) {
      xSemaphoreTake(req_finished_semaphore, 1);
      if (rx_request.state != IORequestState::Busy && tx_request.state != IORequestState::Busy) {
       return;
      }
      if(timeout != -1 && xTaskGetTickCount() > tick_count_timeout )
      {
    	  rx_functions->abort_request(&rx_request, device_index);
    	  tx_functions->abort_request(&tx_request, device_index);
    	  return ;
      }
    }
}

size_t IODevice::read(uint8_t* buffer, size_t buffer_size, uint32_t timeout) {
  if (buffer_size == 0) {
    return 0;
  }

  if (io_flags & IODeviceFlags::RxBlocking) {
    // blocking io
    rx_request.rx_ptr = buffer;
    rx_request.size = buffer_size;
    rx_request.callback = &io_device_rx_complete_callback_blocking;
    rx_functions->start_request(&rx_request, device_index);
    auto tick_count_timeout = xTaskGetTickCount() + timeout;
    while (true) {
      xSemaphoreTake(req_finished_semaphore, 1);
      if (rx_request.state == IORequestState::Complete) {
        rx_request.state = IORequestState::Ready;
        return rx_request.size - rx_request.remaining;
      }
      if (rx_request.state == IORequestState::Error) {
        rx_request.state = IORequestState::Ready;
        return 0;
      }
      if(timeout != -1 && xTaskGetTickCount() > tick_count_timeout )
      {
    	  rx_functions->abort_request(&rx_request, device_index);
    	  return rx_request.size - rx_request.remaining;
      }
    }
  } else  // non blocking io
  {
    // Update state of rx request to read received data without waiting for end
    // of current transfer.
	taskENTER_CRITICAL();
	rx_functions->update_request(&rx_request, device_index);
	rx_queue.unmap_push(rx_request.rx_ptr, rx_request.size - rx_request.remaining);
	taskEXIT_CRITICAL();
    auto bytes_read = rx_queue.pop(buffer, buffer_size);
    try_start_rx_fifo();
    return bytes_read;
  }
  return 0;
}

size_t IODevice::write(const uint8_t* buffer, size_t buffer_size) {
  if (buffer_size == 0) {
    return 0;
  }
  if (io_flags & IODeviceFlags::TxBlocking) {
    // blocking io
    tx_request.tx_ptr = (uint8_t*)buffer;
    tx_request.size = buffer_size;
    tx_request.callback = &io_device_tx_complete_callback_blocking;
    tx_request.userdata = this;
    tx_functions->start_request(&tx_request, device_index);

    while (true) {
      xSemaphoreTake(req_finished_semaphore, portMAX_DELAY);
      if (tx_request.state == IORequestState::Complete) {
        tx_request.state = IORequestState::Ready;
        return tx_request.size - tx_request.remaining;
      }
      if (tx_request.state == IORequestState::Error) {
        tx_request.state = IORequestState::Ready;
        return 0;
      }
    }
  } else {
    // non blocking io
    // Update state of tx request to get an accurate measure of space remaining
    // in the tx queue.
    taskENTER_CRITICAL();
    tx_functions->update_request(&tx_request, device_index);
    tx_queue.unmap_pop(tx_request.tx_ptr, tx_request.size - tx_request.remaining);
    taskEXIT_CRITICAL();

    uint16_t bytes_written = tx_queue.push(buffer, buffer_size);

    // If not currently transmitting, start tx request
    if (tx_request.state == IORequestState::Ready) {
      start_tx_fifo();
    }
    return bytes_written;
  }
  return 0;
}

size_t IODevice::map_read(uint8_t** buffer) {
  if (io_flags & IODeviceFlags::RxBlocking) {
    // mmapped io only works in fifo mode
    *buffer = nullptr;
    return 0;

  } else {
    // Update state of rx request to read received data without waiting for end
    // of current transfer.
    schedule_callback(2);
    auto retval = rx_queue.map_pop(buffer);
    return retval;
  }
}

void IODevice::unmap_read(uint8_t* buffer, size_t size) {
  if (size != 0) {
    rx_queue.unmap_pop(buffer, size);
  }
  try_start_rx_fifo();
}

size_t IODevice::map_write(uint8_t** buffer) {
  if (io_flags & IODeviceFlags::TxBlocking) {
    // mmapped io only works in fifo mode
    *buffer = nullptr;
    return 0;

  } else {
    // Update state of tx request to get an accurate measure of space remaining
    // in the tx queue.
    taskENTER_CRITICAL();
    if (tx_request.state == IORequestState::Busy) {
      tx_functions->update_request(&tx_request, device_index);
      tx_queue.unmap_pop(tx_request.tx_ptr, tx_request.size - tx_request.remaining);
    }
    taskEXIT_CRITICAL();
    return tx_queue.map_push(buffer);
  }
}

void IODevice::unmap_write(uint8_t* buffer, size_t size) {
  tx_queue.unmap_push(buffer, size);

  // If not currently transmitting, start tx request
  if (tx_request.state == IORequestState::Ready) {
    start_tx_fifo();
  }
}

void IODevice::schedule_callback(uint8_t callback_id) {
  hal_callback_send(
      HalCallback{DeviceType::IODevice, static_cast<uint8_t>(this - g_io_devices), callback_id});
}

bool IODevice::queue_rx_request(uint8_t* buffer, size_t size, IORequestCallback callback) {
  if (rx_request.state.load() != IORequestState::Ready || size == 0) {
    return false;
  }
  rx_request.rx_ptr = buffer;
  rx_request.size = size;
  rx_request.callback = callback;
  rx_request.state.store(IORequestState::Pending);
  return true;
}

void hal_iodevice_callback(int id, int callback_id) {
  IODevice* device = &g_io_devices[id];
}

}  // namespace platform
using namespace platform;

static void check_io_device_id(int id) {
  assert(id >= 0 && (unsigned)id < sizeof(g_io_devices) / sizeof(IODevice));
}

void io_execute(int id, IORequest request, uint32_t timeout)
{
	check_io_device_id(id);
	g_io_devices[id].execute(request, timeout);
}

size_t io_read(int id, uint8_t* buffer, size_t buffer_size, uint32_t timeout) {
  check_io_device_id(id);
  return g_io_devices[id].read(buffer, buffer_size, timeout);
}

size_t io_write(int id, const uint8_t* buffer, size_t buffer_size) {
  check_io_device_id(id);
  return g_io_devices[id].write(buffer, buffer_size);
}

size_t io_map_read(int id, uint8_t** buffer) {
  check_io_device_id(id);
  return g_io_devices[id].map_read(buffer);
}

void io_unmap_read(int id, uint8_t* buffer, size_t size) {
  check_io_device_id(id);
  g_io_devices[id].unmap_read(buffer, size);
}

size_t io_map_write(int id, uint8_t** buffer) {
  check_io_device_id(id);
  return g_io_devices[id].map_write(buffer);
}

void io_unmap_write(int id, uint8_t* buffer, size_t size) {
  check_io_device_id(id);
  g_io_devices[id].unmap_write(buffer, size);
}

size_t io_read_bytes_available(int id) {
  check_io_device_id(id);
  return g_io_devices[id].rx_queue.size();
}

size_t io_write_space_available(int id) {
  auto& device = g_io_devices[id];
  return device.tx_queue.space_available();
}

}  // namespace hal
}  // namespace goldobot
