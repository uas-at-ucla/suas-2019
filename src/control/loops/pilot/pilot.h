#ifndef SPINNY_CONTROL_LOOPS_FLIGHT_LOOP_PILOT_PILOT_H_
#define SPINNY_CONTROL_LOOPS_FLIGHT_LOOP_PILOT_PILOT_H_

#include <vector>
#include <map>
#include <string>

#include "lib/physics_structs/physics_structs.h"

namespace spinny {
namespace control {
namespace loops {
namespace pilot {

class Pilot {
 public:
  Pilot();

  Vector3D Calculate(Position3D drone_position, Position3D goal);

 private:
//::std::map<::std::string, int> commands_;
//::std::vector<::std::string> command_order_;
//int command_pointer_;
};

}  // namespace pilot
}  // namespace loops
}  // namespace control
}  // namespace spinny

#endif  // SPINNY_CONTROL_LOOPS_FLIGHT_LOOP_PILOT_PILOT_H_
