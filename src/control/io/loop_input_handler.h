#ifndef SPINNY_CONTROL_IO_LOOP_INPUT_HANDLER_H_
#define SPINNY_CONTROL_IO_LOOP_INPUT_HANDLER_H_

#include <atomic>

#include "aos/common/util/phased_loop.h"

namespace spinny {
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
}  // namespace spinny

#endif  // SPINNY_CONTROL_IO_LOOP_INPUT_HANDLER_H_
