#pragma once
#include <cstdint>

namespace goldobot {
struct PropulsionCommand {
  enum CommandType { Move };
};

class PropulsionCommandQueue {
 public:
  PropulsionCommandQueue();
  ~PropulsionCommandQueue();

  //! \brief Return the number of free slots in the queue
  unsigned freeSlots();

  //! \brief Return currently executing command or nullptr if the queue is empty
  PropulsionCommand* current() const;

  //! \brief Push new command at the back of the queue, return true on success, false if queue was
  //! full.
  bool push(const PropulsionCommand&& command);

  //! \brief Pop current command from the queue and return pointer to new current command, or
  //! nullptr if the queue is empty
  PropulsionCommand* pop();

  //! \brief Remove all commands from queue
  void clear();

 private:
};

}  // namespace goldobot
