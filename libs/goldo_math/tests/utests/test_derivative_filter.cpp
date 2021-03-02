#include "goldobot/core/derivative_filter.hpp"

#include "gtest/gtest.h"

TEST(DerivativeFilterTest, Negative) {
  goldobot::DerivativeFilter filter;
  filter.setConfig(1e-3f, 10);
  filter.reset(1.0f);
  EXPECT_NEAR(filter.step(1e-3f), 1.0f, 1e-3f);
}
