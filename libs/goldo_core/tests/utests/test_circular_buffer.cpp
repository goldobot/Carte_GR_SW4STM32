#include "goldobot/core/circular_buffer.hpp"

#include "gtest/gtest.h"

TEST(CircularBufferTest, Constructor) {
  uint8_t buff[128];
  goldobot::CircularBuffer<> circular_buffer;
  circular_buffer.init(buff, sizeof(buff));

  EXPECT_EQ(circular_buffer.size(), 0);
  EXPECT_EQ(circular_buffer.spaceAvailable(), circular_buffer.maxSize());
  EXPECT_GE(circular_buffer.maxSize(), sizeof(buff) - 1);
}

TEST(CircularBufferTest, PopEmpty) {
  uint8_t buff[128];
  goldobot::CircularBuffer<> circular_buffer;
  circular_buffer.init(buff, sizeof(buff));

  uint8_t pop_buff[16];

  EXPECT_EQ(circular_buffer.pop(pop_buff, sizeof(pop_buff)), 0);
}

TEST(CircularBufferTest, MapPopEmpty) {
  uint8_t buff[128];
  goldobot::CircularBuffer<> circular_buffer;
  circular_buffer.init(buff, sizeof(buff));

  uint8_t* pop_buff;

  EXPECT_EQ(circular_buffer.mapPop(&pop_buff), 0);
}