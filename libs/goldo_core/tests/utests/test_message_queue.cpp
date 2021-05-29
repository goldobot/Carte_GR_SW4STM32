#include "goldobot/core/message_queue.hpp"

#include "gtest/gtest.h"

#include <random>

TEST(MessageQueueTest, Test) {
  std::default_random_engine e1(42);

  uint8_t buff[64];
  goldobot::MessageQueue message_queue(buff, sizeof(buff));

  uint8_t tmp_buff_1[64];
  uint8_t tmp_buff_2[64];

  for (int i = 0; i < 64; i++) {
    tmp_buff_1[i] = e1() % 0xff;
  }

  int b = message_queue.available_capacity();

  message_queue.push_message(static_cast<goldobot::CommMessageType>(42), tmp_buff_1, 16);
  message_queue.push_message(static_cast<goldobot::CommMessageType>(43), tmp_buff_1, 16);
  message_queue.push_message(static_cast<goldobot::CommMessageType>(44), tmp_buff_1, 16);
  message_queue.push_message(static_cast<goldobot::CommMessageType>(45), tmp_buff_1, 3);

  ASSERT_EQ(message_queue.message_type(), static_cast<goldobot::CommMessageType>(42));
  message_queue.pop_message(tmp_buff_2, 32);
  message_queue.push_message(static_cast<goldobot::CommMessageType>(46), tmp_buff_1, 16);

  message_queue.pop_message(tmp_buff_2, 16);
  message_queue.pop_message(tmp_buff_2, 16);
  message_queue.pop_message(tmp_buff_2, 16);
  message_queue.pop_message(tmp_buff_2, 16);
  message_queue.pop_message(tmp_buff_2, 16);

  int c = message_queue.available_capacity();
  int a = 1;

  // EXPECT_EQ(circular_buffer.pop(pop_buff, sizeof(pop_buff)), 0);
}
