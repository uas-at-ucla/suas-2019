#pragma once

#include <atomic>

#include "lib/phased_loop/phased_loop.h"

namespace src {
namespace control {
namespace io {

class LoopInputHandler {
 public:
  LoopInputHandler();
  void operator()();
  void Quit() { run_ = false; }

 protected:
  ::lib::phased_loop::PhasedLoop phased_loop_;

 private:
  virtual void RunIteration() = 0;

  ::std::atomic<bool> run_{true};
};

}  // namespace io
}  // namespace control
}  // namespace src

