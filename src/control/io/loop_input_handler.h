#pragma once

#include <atomic>

#include "aos/common/util/phased_loop.h"

namespace src {
namespace control {
namespace io {

class LoopInputHandler {
 public: 
  LoopInputHandler();
  void operator()();
  void Quit() { run_ = false; }

 protected:
  aos::time::PhasedLoop phased_loop_;

 private:
  virtual void RunIteration() = 0;

  ::std::atomic<bool> run_{true};
};

}  // namespace io
}  // namespace control
}  // namespace src

