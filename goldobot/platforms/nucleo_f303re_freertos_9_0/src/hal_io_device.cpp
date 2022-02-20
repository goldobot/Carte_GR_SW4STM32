
#include "goldobot/platform/hal_io_device.hpp"

#include "goldobot/platform/hal_i2c.hpp"
#include "goldobot/platform/hal_spi.hpp"
#include "goldobot/platform/hal_uart.hpp"

#include <cassert>

namespace goldobot {
namespace hal {
namespace platform {

bool default_start_request(IORequest* req, uint32_t device_index) {
  assert(false);
  return false;
}

bool default_update_request(IORequest* req, uint32_t device_index) {
  assert(false);
  return false;
}

bool default_abort_request(IORequest* req, uint32_t device_index) {
  assert(false);
  return false;
}

IODeviceFunctions g_default_device_functions = {&default_start_request, &default_update_request,
                                                default_abort_request};

IODevice g_io_devices[8];

void init_io_devices() {
  for (unsigned i = 0; i < 8; i++) {
    g_io_devices[i].rx_functions = &g_default_device_functions;
    g_io_devices[i].tx_functions = &g_default_device_functions;
  }
}

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

  device->rx_semaphore = xSemaphoreCreateBinary();
  device->tx_semaphore = xSemaphoreCreateBinary();

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
  if (config->rx_buffer_size > 0 && !(device->io_flags & IODeviceFlags::RxBlocking)) {
    device->try_start_rx_fifo();
  }
}

bool io_device_rx_complete_callback_fifo(IORequest* req, IORequestStatus status) {
  auto device = reinterpret_cast<IODevice*>(req->userdata);
  device->rx_queue.unmap_push(req->rx_ptr, req->size - req->remaining);

  if (status == IORequestStatus::Success) {
    uint8_t* ptr;
    auto size = device->rx_queue.map_push(&ptr);
    if (size > 0) {
      req->rx_ptr = ptr;
      req->size = size;
      return true;
    } else {
      device->rx_busy = false;
      return false;
    }
  }
  if (status == IORequestStatus::Error) {
    device->rx_busy = false;
    return false;
  }
  return false;
}

bool io_device_tx_complete_callback_fifo(IORequest* req, IORequestStatus status) {
  auto device = reinterpret_cast<IODevice*>(req->userdata);

  device->tx_queue.unmap_pop(req->tx_ptr, req->size - req->remaining);

  if (status == IORequestStatus::Success) {
    uint8_t* ptr;
    auto size = device->tx_queue.map_pop(&ptr);
    if (size > 0) {
      req->tx_ptr = ptr;
      req->size = size;
      return true;
    } else {
      device->tx_busy = false;
      return false;
    }
  }
  if (status == IORequestStatus::Error) {
    device->tx_busy = false;
    return false;
  }

  return false;
}

void IODevice::try_start_rx_fifo() {
  if (rx_busy) {
    return;
  }

  uint8_t* ptr;
  auto size = rx_queue.map_push(&ptr);

  if (size > 0) {
    rx_request.rx_ptr = ptr;
    rx_request.size = size;
    rx_request.callback = &io_device_rx_complete_callback_fifo;
    rx_request.userdata = this;
    rx_busy = true;
    if (!rx_functions->start_request(&rx_request, device_index)) {
      rx_busy = false;
    }
  }
}

/*
bool io_device_rx_complete_callback_blocking(IORequest* req, IODevice* device) {
 // assert(req->state == IORequestState::Complete);
  xSemaphoreGive(device->rx_semaphore);
}

bool io_device_tx_complete_callback_blocking(IORequest* req, IODevice* device) {
 // assert(req->state == IORequestState::Complete);
  xSemaphoreGive(device->tx_semaphore);
}*/

void io_device_tx_complete_callback_execute(IORequest* req, IODevice* device) {}

void io_device_rx_complete_callback_execute(IORequest* req, IODevice* device) {}

void IODevice::execute(IORequest* req, uint32_t timeout) {
  if (req->rx_ptr != nullptr) {
    rx_functions->start_request(req, device_index);
  } else if (req->tx_ptr != nullptr) {
    tx_functions->start_request(req, device_index);
  }

  auto tick_count_timeout = xTaskGetTickCount() + timeout;
  while (true) {
    xSemaphoreTake(rx_semaphore, 1);

    if (timeout != -1 && xTaskGetTickCount() > tick_count_timeout) {
      rx_functions->abort_request(nullptr, device_index);
      tx_functions->abort_request(nullptr, device_index);
      return;
    }
  }
}

void IODevice::reset() {
  rx_functions->abort_request(nullptr, device_index);
  tx_functions->abort_request(nullptr, device_index);
  tx_busy = false;
  rx_busy = false;
  try_start_rx_fifo();
}

size_t IODevice::read(uint8_t* buffer, size_t buffer_size, uint32_t timeout) {
  if (buffer_size == 0) {
    return 0;
  }

  if (io_flags & IODeviceFlags::RxBlocking) {
    // blocking io
    // rx_request.rx_ptr = buffer;
    // rx_request.size = buffer_size;
    // rx_request.callback = &io_device_rx_complete_callback_blocking;
    schedule_callback(0);
    while (true) {
      xSemaphoreTake(rx_semaphore, portMAX_DELAY);
      //  if (rx_request.state == IORequestState::Complete) {
      //    rx_request.state = IORequestState::Ready;
      //    return rx_request.size - rx_request.remaining;
      //  }
      //  if (rx_request.state == IORequestState::Error) {
      //    rx_request.state = IORequestState::Ready;
      //     return 0;
      //   }
    }
  } else  // non blocking io
  {
    if (rx_busy) {
      rx_functions->update_request(nullptr, device_index);
    } else {
      try_start_rx_fifo();
    }
    return rx_queue.pop(buffer, buffer_size);
  }
  return 0;
}

size_t IODevice::write(const uint8_t* buffer, size_t buffer_size) {
  if (buffer_size == 0) {
    return 0;
  }
  if (io_flags & IODeviceFlags::TxBlocking) {
  } else {
    // non blocking io
    if (tx_busy) {
      tx_functions->update_request(nullptr, device_index);
      return tx_queue.push(buffer, buffer_size);
    } else {
      uint16_t bytes_written = tx_queue.push(buffer, buffer_size);

      uint8_t* ptr;
      auto size = tx_queue.map_pop(&ptr);

      if (size > 0) {
        tx_request.tx_ptr = ptr;
        tx_request.size = size;
        tx_request.callback = &io_device_tx_complete_callback_fifo;
        tx_request.userdata = this;
        tx_busy = true;
        if (!tx_functions->start_request(&tx_request, device_index)) {
          tx_busy = false;
        }
      }
      return bytes_written;
    }
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
    rx_functions->update_request(nullptr, device_index);
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
    // mapped io only works in fifo mode
    *buffer = nullptr;
    return 0;

  } else {
    // Update state of tx request to get an accurate measure of space remaining
    // in the tx queue.
    tx_functions->update_request(nullptr, device_index);
  }
  return tx_queue.map_push(buffer);
}

void IODevice::unmap_write(uint8_t* buffer, size_t size) {
  tx_queue.unmap_push(buffer, size);

  if (!tx_busy) {
    uint8_t* ptr;
    auto size = tx_queue.map_pop(&ptr);

    if (size > 0) {
      tx_request.tx_ptr = ptr;
      tx_request.size = size;
      tx_request.callback = &io_device_tx_complete_callback_fifo;
      tx_request.userdata = this;
      tx_busy = true;
      tx_functions->start_request(&tx_request, device_index);
    }
  }
}

void IODevice::schedule_callback(uint8_t callback_id) {
  hal_callback_send(
      HalCallback{DeviceType::IODevice, static_cast<uint8_t>(this - g_io_devices), callback_id});
}

void hal_iodevice_callback(int id, int callback_id) { IODevice* device = &g_io_devices[id]; }

}  // namespace platform
using namespace platform;

static void check_io_device_id(int id) {
  assert(id >= 0 && (unsigned)id < sizeof(g_io_devices) / sizeof(IODevice));
}

void io_execute(int id, IORequest* request, uint32_t timeout) {
  check_io_device_id(id);
  g_io_devices[id].execute(request, timeout);
}

void io_reset(int id) {
  check_io_device_id(id);
  g_io_devices[id].reset();
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
