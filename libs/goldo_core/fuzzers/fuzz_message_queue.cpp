#include "goldobot/core/message_queue.hpp"
#include <algorithm>
#include <vector>
#include <cassert>

#include <iostream>

struct Command {
  enum class Type { Push, Pop };
  Type type;
  size_t size;
};

extern "C" int LLVMFuzzerTestOneInput(const uint8_t* Data, size_t Size) {
  if (Size < 128) {
    return 0;
  };

  // Prepare list of messages and commands
  std::vector<std::tuple<goldobot::CommMessageType, const uint8_t*, size_t>> messages;
  std::vector<Command> commands;
  size_t sum_message_sizes{0};

  const uint8_t* ptr = Data;
  const uint8_t* ptr_end = Data + Size;
  int check_msg_idx{0};  // index of next message to read

  constexpr size_t msg_size_max = 128;

  // pick commands
  // read commands as (type, size) tuples from the start of the buffer
  // messages data is allocated from the end of the buffer
  // stop when there is no more space to allocate a 1 byte message for the next command

  while (ptr_end - ptr - sum_message_sizes > sizeof(size_t) + sizeof(uint8_t) + sizeof(goldobot::CommMessageType)) {
    // choose a command type and size
    uint8_t cmd_choice = *ptr++;
    goldobot::CommMessageType msg_type;
    memcpy(&msg_type, ptr, sizeof(msg_type));
    ptr += sizeof(msg_type);
    size_t msg_size{0};
    memcpy(&msg_size, ptr, sizeof(size_t));
    msg_size = (msg_size % (msg_size_max - 1)) + 1;
    ptr += sizeof(size_t);

    if (cmd_choice < 196) {
      // push message
      auto remaining_space = ptr_end - ptr - sum_message_sizes;
      msg_size = std::min(msg_size, remaining_space);

      // message pointers whil be filled later
      messages.push_back(std::make_tuple(msg_type, (const uint8_t*) nullptr, msg_size));
      commands.push_back(Command{Command::Type::Push, msg_size});
      sum_message_sizes += msg_size;
    } else {
      // pop message
      commands.push_back(Command{Command::Type::Pop, msg_size});
    }
  }

  for (auto& msg : messages) {
    std::get<1>(msg) = ptr;
    ptr += std::get<2>(msg);
    assert(ptr <= ptr_end);
  }

  uint8_t buff[1024];
  goldobot::MessageQueue message_queue(buff, sizeof(buff));
  uint8_t msg_buffer[msg_size_max];

  unsigned msg_idx_head{0};
  unsigned msg_idx_tail{0};

  for (auto cmd : commands) {
    switch (cmd.type) {
      case Command::Type::Push: {
        auto& msg = messages[msg_idx_head];
        if (message_queue.push_message(std::get<0>(msg), std::get<1>(msg), std::get<2>(msg)))
        {
            // successfully pushed message
            msg_idx_head++;
        }
       
      } break;
      case Command::Type::Pop: {        
        if (msg_idx_tail == msg_idx_head) {
          assert(!message_queue.message_ready());
        } else {
          auto& msg = messages[msg_idx_tail];

          assert(message_queue.message_type() == std::get<0>(msg));
          assert(message_queue.message_size() == std::get<2>(msg));
          auto pop_size = message_queue.pop_message((unsigned char*)msg_buffer, cmd.size);
          assert(pop_size = std::min(std::get<2>(msg), cmd.size));
          assert(memcmp(msg_buffer, std::get<1>(msg), pop_size) == 0);

          msg_idx_tail++;
        }      
      } break;
      default:
        break;
    }
  }

  /*
  for (auto msg : messages) {
    size_t msg_size = msg.second - msg.first;
    if (msg_size > 100) {
      // std::cout << msg_size << "\n";
    };
    circular_buffer.push_message(static_cast<goldobot::CommMessageType>(42), msg.first,
                                 msg_size);
    assert(circular_buffer.message_size() == msg_size);
    size_t pop_size = circular_buffer.pop_message(msg_buffer, sizeof(msg_buffer));
    assert(pop_size == msg_size);
    assert(memcmp(msg_buffer, msg.first, msg_size) == 0);
  };*/

  return 0;  // Non-zero return values are reserved for future use.
}
