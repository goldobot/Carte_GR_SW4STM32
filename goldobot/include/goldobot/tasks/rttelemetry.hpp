#pragma once
#include <cstdint>

#include "goldobot/platform/task.hpp"

namespace goldobot {
class RtTelemetryTask : public Task {
 public:
  RtTelemetryTask();
  const char* name() const override;

 private:
  void taskFunction() override;
};
}  // namespace goldobot
