#ifndef SPINNY_CONTROL_LOOPS_FLIGHT_LOOP_H_
#define SPINNY_CONTROL_LOOPS_FLIGHT_LOOP_H_

#include "aos/common/util/phased_loop.h"
#include "aos/common/time.h"

#include "src/control/loops/flight_loop.q.h"

namespace spinny {
namespace control {
namespace loops {

class FlightLoop {
 public:
  FlightLoop();

  void Run();

  enum State {
    UNINITIALIZED = 0,
    STANDBY = 1,
    ARMING = 2,
    ARMED = 3,
    TAKING_OFF = 4,
    IN_AIR = 5,
    LANDING = 6,
    FAILSAFE = 7,
    FLIGHT_TERMINATION = 8
  };

  State state() const {
    return state_;
  }

 private:
  void RunIteration();

  State state_;

  bool running_;

  aos::time::PhasedLoop phased_loop_;
  std::chrono::time_point<std::chrono::system_clock>  start_;
};

}  // namespace loops
}  // namespace control
}  // namespace spinny

#endif  // SPINNY_CONTROL_LOOPS_FLIGHT_LOOP_H_
