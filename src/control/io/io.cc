#include "io.h"

namespace spinny {
namespace control {
namespace io {

IO::IO() {}

void IO::Run() { RunAutopilotIO(); }

void IO::RunAutopilotIO() {
  copter_io_.start();

  for (int i = 0; i < 25 || true; i++) {
    mavlink_local_position_ned_t pos =
        copter_io_.current_messages.local_position_ned;
    printf("%i CURRENT POSITION XYZ = [ % .4f , % .4f , % .4f ] \n", i, pos.x,
           pos.y, pos.z);
    usleep(1e6 / 20);
  }

  copter_io_.stop();
}

}  // namespace io
}  // namespace control
}  // namespace spinny
