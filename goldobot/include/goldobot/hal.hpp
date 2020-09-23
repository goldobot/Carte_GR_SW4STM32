#pragma once
#include <cstddef>
#include <cstdint>

namespace goldobot {
namespace hal {

typedef uint32_t TickType_t;

enum class Status { Error = 0, Ok = 1 };

void init();

//! \brief Configure peripherals
void configure(uint8_t* config);

TickType_t get_tick_count();

bool gpio_get(int gpio_id);
void gpio_set(int gpio_id, bool value);

void pwm_set(int pwm_id, float value);

uint16_t encoder_get(int encoder_id);
uint16_t encoder_set(int encoder_id, uint16_t value);

size_t io_read(int id, uint8_t* buffer, size_t size);
size_t io_write(int id, const uint8_t* buffer, size_t size);
size_t io_readwrite(int id, uint8_t read_buffer, const uint8_t* write_buffer, size_t size);

size_t io_map_read(int id, uint8_t** buffer);
size_t io_unmap_read(int id, uint8_t* buffer, size_t size);

size_t io_map_write(int id, uint8_t** buffer);
size_t io_unmap_write(int id, uint8_t* buffer, size_t size);

size_t io_write_space_available(int id);
size_t io_read_bytes_available(int id);

static Status i2c_memory_read(int fd, uint16_t dev_address, uint16_t mem_address,
                              uint16_t mem_address_size, uint8_t* buffer, uint16_t size);
static Status i2c_memory_write(int fd, uint16_t dev_address, uint16_t mem_address,
                               uint16_t mem_address_size, const uint8_t* buffer, uint16_t size);
}  // namespace hal
};  // namespace goldobot
