#include <vector>
#include <cstdint>

std::vector<std::pair<const uint8_t*, size_t>> genMessages(const uint8_t *Data, size_t Size, uint16_t msg_size_max)
{
    std::vector<std::pair<const uint8_t*, size_t>> messages;
    
  if(Size <3)
  {
	  return messages;
  };
  
  
  const uint8_t* ptr = Data;
  const uint8_t* ptr_end = Data + Size;
  
  uint16_t msg_size = msg_size_max > 1 ? (*(uint16_t*)ptr % (msg_size_max - 1)) + 1 : msg_size_max;
  
  while(ptr + 2 + msg_size < ptr_end)
  {	
	messages.push_back(std::make_pair(ptr+2, msg_size));
	
	ptr += 2 + msg_size;	
	if(ptr + 2 >= ptr_end)
	{
		break;
	};
	msg_size = msg_size_max > 1 ? (*(uint16_t*)ptr % (msg_size_max - 1)) + 1 : msg_size_max;
  }
  
  return messages;    
};