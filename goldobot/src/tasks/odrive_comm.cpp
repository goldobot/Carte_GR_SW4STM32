#include "goldobot/tasks/odrive_comm.hpp"

#include <stdarg.h>
#include <stdio.h>

#include <cmath>
#include <cstdlib>
#include <cstring>

#include "goldobot/hal.hpp"
#include "goldobot/platform/message_queue.hpp"
#include "goldobot/propulsion/odometry_config.hpp"
#include "goldobot/robot.hpp"
#include "goldobot/tasks/main.hpp"

using namespace goldobot;

unsigned char __attribute__((section(".ccmram"))) ODriveCommTask::s_message_queue_buffer[1024];
unsigned char __attribute__((section(".ccmram"))) ODriveCommTask::s_write_buffer[256];
unsigned char __attribute__((section(".ccmram"))) ODriveCommTask::s_parse_buffer[256];
unsigned char __attribute__((section(".ccmram"))) ODriveCommTask::s_scratchpad[128];

ODriveCommTask::ODriveCommTask()
    : m_message_queue(s_message_queue_buffer, sizeof(s_message_queue_buffer)),
	  m_stream_writer(s_write_buffer, sizeof(s_write_buffer)),
	  m_stream_parser(s_parse_buffer, sizeof(s_parse_buffer))
	  {}


const char* ODriveCommTask::name() const { return "odrive_comm"; }

void ODriveCommTask::taskFunction() {
  set_priority(5);

  Robot::instance().mainExchangeIn().subscribe(
  	      {410, 420, &m_message_queue});  // odrive messages

  while (1) {
	  {
		// Send serialized data
		uint8_t* ptr;
		size_t space_available = hal::io_map_write(1, &ptr);
		size_t dtlen = m_stream_writer.popData(ptr, space_available);
		hal::io_unmap_write(1, ptr, dtlen);
	  }
	  // Parse received data
	  {
		uint8_t* ptr;
		size_t bytes_available = hal::io_map_read(1, &ptr);
		size_t dtlen = m_stream_parser.pushData((unsigned char*)ptr, bytes_available);
		hal::io_unmap_read(1, ptr, dtlen);
	  }

	  if(m_stream_parser.packetReady())
	  {
		  size_t packet_size = m_stream_parser.packetSize();
		  m_stream_parser.popPacket(s_scratchpad, sizeof(s_scratchpad));
		  Robot::instance().mainExchangeOut().pushMessage(CommMessageType::ODrivePacket, s_scratchpad, packet_size);
	  }

	  // Process messages
	    while (m_message_queue.message_ready()) {
	      processMessage();
	    }

    // Wait for next tick
    delay(1);
  }
}

void ODriveCommTask::processMessage() {
  auto message_type = (CommMessageType)m_message_queue.message_type();

  switch (message_type) {
    case CommMessageType::ODrivePacket:
    {
      auto packet_size = m_message_queue.message_size();
      m_message_queue.pop_message(s_scratchpad, 128);
      m_stream_writer.pushPacket((unsigned char*)s_scratchpad, packet_size);
    }
      break;
    default:
      m_message_queue.pop_message(nullptr, 0);
      break;
  }
}
