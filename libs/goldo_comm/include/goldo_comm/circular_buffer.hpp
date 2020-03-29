#ifndef _GOLDO_COMM__CIRCULAR_BUFFER__HPP_
#define _GOLDO_COMM__CIRCULAR_BUFFER__HPP_
#include <array>
#include <cstdint>
#include <cstring>

namespace goldo_comm {
template <size_t BufferSize>
class CircularBuffer {
 public:
  using buffer = std::array<uint8_t, BufferSize>;
  using iterator = typename buffer::iterator;

 public:
  CircularBuffer()
      : m_read_ptr(m_buffer.begin()),
        m_write_ptr(m_buffer.begin()),
        m_full(false){};

  bool empty() const noexcept { return m_read_ptr == m_write_ptr && !m_full; }
  bool full() const noexcept { return m_full; }

  size_t size() const noexcept {
    if (m_write_ptr == m_read_ptr) {
      return m_full ? m_buffer.size() : 0;
    } else if (m_write_ptr > m_read_ptr) {
      return m_write_ptr - m_read_ptr;
    } else {
      return m_buffer.size() - (m_read_ptr - m_write_ptr);
    }
  }

  size_t max_size() const noexcept { return m_buffer.max_size(); }

  size_t available_space() const noexcept { return max_size() - size(); }

  uint8_t* lock_read(size_t& buffer_size)
  {
	  buffer_size = std::min(size(), buffer_size);
	  buffer_size = std::min<size_t>(buffer_size, m_buffer.end() - m_read_ptr);
	  return &*m_read_ptr;
  }

  void unlock_read(size_t push_size)
  {
	  advance_iterator(m_read_ptr, push_size);
	  if (push_size > 0)
	  {
		  m_full = false;
	  }
  }

  uint8_t* lock_write(size_t& buffer_size)
  {
	  buffer_size = std::min(available_space(), buffer_size);
	  buffer_size = std::min<size_t>(buffer_size, m_buffer.end() - m_write_ptr);
	  return &*m_write_ptr;
  }

  void unlock_write(size_t push_size)
  {
	  advance_iterator(m_write_ptr, push_size);
	  if(m_write_ptr == m_read_ptr && push_size != 0)
	  {
		  m_full = true;
	  }
  }

  size_t push(const uint8_t* buffer, size_t buffer_size) noexcept {
    if (buffer_size >= available_space()) {
      // Enough data to fill the buffer
		buffer_size = available_space();
      m_full = true;      
    }
    if (m_buffer.end() - m_write_ptr >= buffer_size) {
      // Can write data in one block
      memcpy(&*m_write_ptr, buffer, buffer_size);
    } else {
      // Data span the buffer end
      memcpy(&*m_write_ptr, buffer, m_buffer.end() - m_write_ptr);
      memcpy(&*m_buffer.begin(), buffer + (m_buffer.end() - m_write_ptr),
             buffer_size - (m_buffer.end() - m_write_ptr));
    }
    advance_iterator(m_write_ptr, buffer_size);

    return buffer_size;
  }

  size_t peek(uint8_t* buffer, size_t buffer_size) const noexcept {
    buffer_size = std::min(buffer_size, size());

    if (m_buffer.end() - m_read_ptr >= buffer_size) {
      // Can read in one block
      memcpy(buffer, &*m_read_ptr, buffer_size);
    } else {
      // Data span the buffer end
	  size_t s1 = m_buffer.end() - m_read_ptr;
      memcpy(buffer, &*m_read_ptr, s1 );
      memcpy(buffer + s1, &*m_buffer.begin(), buffer_size - s1);
    }
    return buffer_size;
  };

  size_t pop(size_t pop_size) noexcept {
    pop_size = std::min(pop_size, size());
    advance_iterator(m_read_ptr, pop_size);
    if (m_full && pop_size > 0) {
      m_full = false;
    }
    return pop_size;
  };

  size_t pop(uint8_t* buffer, size_t buffer_size) noexcept {
    return pop(peek(buffer, buffer_size));
  };

// private:
  buffer m_buffer;
  iterator m_read_ptr;
  iterator m_write_ptr;
  bool m_full{false};

  void advance_iterator(iterator& it, size_t size) {
	  ptrdiff_t diff = size;
	  while (diff >= m_buffer.end() - it)
	  {
		  diff -= max_size();
	  }
    it += diff;   
  }
};
}  // namespace goldo_comm

#endif  // _GOLDO_COMM__CIRCULAR_BUFFER__HPP_
