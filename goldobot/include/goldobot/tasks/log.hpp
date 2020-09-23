#pragma once

namespace goldobot {
enum class LogPriority { Info = 0, Debug, Warning, Error, CriticalError };

class LogTask {
 public:
  LogTask();
  void logMessage(LogPriority priority, const char* task_name, const char* message);
};
}  // namespace goldobot
