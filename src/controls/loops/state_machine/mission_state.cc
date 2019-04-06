#include "state_machine.h"

namespace src {
namespace controls {
namespace loops {
namespace state_machine {

MissionState::MissionState() {}

void MissionState::Handle(::src::controls::Sensors &sensors,
                          ::src::controls::Goal &goal,
                          ::src::controls::Output &output) {
  (void)goal;

  lib::Position3D position = {sensors.latitude(), sensors.longitude(),
                              sensors.relative_altitude()};

  ::Eigen::Vector3d velocity(sensors.velocity_x(), sensors.velocity_y(),
                             sensors.velocity_z());

  executor::ExecutorOutput executor_output =
      executor_.Calculate(position, velocity);

  // last_bomb_drop_ =
  //     executor_output.bomb_drop ? sensors.time() : last_bomb_drop_;

  if (executor_output.alarm) {
    // alarm_.AddAlert({5.0, 0.50});
  }

  output.set_velocity_x(executor_output.flight_velocities.x);
  output.set_velocity_y(executor_output.flight_velocities.y);
  output.set_velocity_z(executor_output.flight_velocities.z);
  output.set_yaw_setpoint(executor_output.yaw);
}

void MissionState::Reset() {}

} // namespace state_machine
} // namespace loops
} // namespace controls
} // namespace src
