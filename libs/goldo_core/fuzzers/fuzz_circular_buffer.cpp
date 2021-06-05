#include "goldo_comm/circular_buffer.hpp"
#include <algorithm>
#include <vector>
#include <cassert>

#include <iostream>


extern "C" int LLVMFuzzerTestOneInput(const uint8_t *Data, size_t Size) {

  if(Size <3)
  {
	  return 0;
  };
  
  //Prepare list of messages to send
  std::vector<std::pair<const uint8_t*, const uint8_t*>> messages;
  
  
  const uint8_t* ptr = Data;
  const uint8_t* ptr_end = Data + Size;
  
  uint16_t msg_size_max = 128;
  uint16_t msg_size = (*(uint16_t*)ptr % (msg_size_max - 1)) + 1;
  
  while(ptr + 2 + msg_size < ptr_end)
  {	
	messages.push_back(std::make_pair(ptr+2, ptr+2+msg_size));
	
	ptr += 2 + msg_size;	
	if(ptr + 2 >= ptr_end)
	{
		break;
	};
	msg_size = (*(uint16_t*)ptr % (msg_size_max - 1)) + 1;	
  }
  
  goldo_comm::CircularBuffer<256> circular_buffer;
  uint8_t msg_buffer[128];
  
  for(auto msg : messages)
  {
	  size_t msg_size = msg.second-msg.first;
	  if(msg_size > 100)
	  {
		  //std::cout << msg_size << "\n";
	  };
	  size_t pushed_size = circular_buffer.push(msg.first, msg_size);
	  assert(pushed_size == msg_size);
	  assert(circular_buffer.size() == msg_size);
	  assert(circular_buffer.available_space() == 256 - msg_size);
	  
	  size_t pop_size = circular_buffer.pop(msg_buffer, 128);
	  assert(pop_size == msg_size);
	  assert(memcmp(msg_buffer, msg.first, msg_size) == 0);
  };

  return 0;  // Non-zero return values are reserved for future use.
}