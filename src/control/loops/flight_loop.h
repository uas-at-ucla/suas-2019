#ifndef SPINNY_CONTROL_LOOPS_FLIGHT_LOOP_H_
#define SPINNY_CONTROL_LOOPS_FLIGHT_LOOP_H_

#include <atomic>

#include "aos/common/util/phased_loop.h"
#include "aos/common/time.h"
#include "aos/linux_code/init.h"

#include "src/control/loops/flight_loop.q.h"

namespace spinny {
namespace control {
namespace loops {

class FlightLoop {
 public:
  FlightLoop();

  void Run();
  void Iterate();

  enum State {
    STANDBY,
    ARMING,
    ARMED,
    TAKING_OFF,
    IN_AIR,
    LANDING,
    FAILSAFE,
    FLIGHT_TERMINATION
  };

  State state() const {
    return state_;
  }

 private:
  void RunIteration();

  void DumpSensors();

  State state_;

  ::std::atomic<bool> running_;

  ::aos::time::PhasedLoop phased_loop_;
  ::std::chrono::time_point<std::chrono::system_clock>  start_;
};

}  // namespace loops
}  // namespace control
}  // namespace spinny

#endif  // SPINNY_CONTROL_LOOPS_FLIGHT_LOOP_H_
