#ifndef SPINNY_CONTROL_IO_IO_H_
#define SPINNY_CONTROL_IO_IO_H_

#include "src/control/io/autopilot_interface/autopilot_interface.h"

#include "src/control/loops/flight_loop.q.h"

namespace spinny {
namespace control {
namespace io {

class IO {
 public:
  IO();
  void Run();

 private:
  autopilot_interface::AutopilotInterface copter_io_;
  void RunAutopilotIO();
};

}  // namespace io
}  // namespace control
}  // namespace spinny

#endif  // SPINNY_CONTROL_IO_IO_H_
