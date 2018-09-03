#pragma once

#include <atomic>
#include <chrono>

#include "lib/phased_loop/phased_loop.h"

namespace src {
namespace control {
namespace io {

class LoopOutputHandler {
 public:
  LoopOutputHandler();

  void Quit() { run_ = false; }

  void operator()();

 protected:
  // Read a message from the appropriate queue.
  // This must block.
  virtual void Read() = 0;

  // Send the output from the appropriate queue to the hardware.
  // Read() will always be called at least once before per invocation of this.
  virtual void Write() = 0;

  // Stop all outputs. This will be called in a separate thread (if at all).
  // The subclass implementation should handle any appropriate logging.
  // This will be called exactly once if Read() blocks for too long and once
  // after Quit is called.
  virtual void Stop() = 0;

 private:
  ::lib::phased_loop::PhasedLoop phased_loop_;

  ::std::atomic<bool> run_{true};
};

} // namespace io
} // namespace control
} // namespace src
