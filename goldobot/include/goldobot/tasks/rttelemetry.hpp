#pragma once
#include "goldobot/platform/task.hpp"

#include <cstdint>

namespace goldobot {
class RtTelemetryTask : public Task {
 public:
  RtTelemetryTask();
  const char* name() const override;

 private:
  void taskFunction() override;
};
}  // namespace goldobot
