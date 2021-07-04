#pragma once
#include "goldobot/platform/lockers.hpp"

#include <cstddef>

namespace goldobot {
enum class CommMessageType : uint16_t;

class MessageQueue {
 public:
  struct Statistics {
    size_t min_available_capacity{0};
    size_t bytes_pushed{0};
    size_t messages_pushed{0};
  };

 public:
  MessageQueue(unsigned char* buffer, size_t size);

  bool message_ready() const;
  CommMessageType message_type() const;
  size_t message_size() const;

  bool push_message(CommMessageType message_type, const unsigned char* buffer, size_t size);
  size_t pop_message(unsigned char* buffer, size_t size);
  bool pop_message(unsigned char** buffers, size_t* sizes, int num_buffers);

  size_t available_capacity() const;

  Statistics statistics();

 private:
  void push_data(const unsigned char* buffer, size_t size);
  void read_data(size_t start_index, unsigned char* buffer, size_t size);
  void pop_data(size_t size);
  size_t buffer_capacity() const noexcept;

  unsigned char* m_buffer;
  size_t m_buffer_size;
  size_t m_begin_index;
  size_t m_end_index;
  bool m_message_ready{false};
  uint16_t m_message_size{0};
  CommMessageType m_message_type{static_cast<CommMessageType>(0)};
  Statistics m_statistics;

  detail::LockerMutex m_mutex;
};
}  // namespace goldobot
