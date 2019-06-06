#pragma once

#include "src/controls/ground_controls/timeline/timeline_grammar.pb.h"
#include "lib/mission_manager/mission_commands.pb.h"
#include "lib/physics_structs/physics_structs.h"
#include "lib/rrt_avoidance/2dplane/2dplane.hpp"
#include "lib/rrt_avoidance/2dplane/GridStateSpace.hpp"
#include "lib/rrt_avoidance/birrt/birrt.h"
#include "lib/rrt_avoidance/planning/Path.hpp"

namespace lib {
namespace rrt_avoidance {

class RRTAvoidance {
 public:
  RRTAvoidance();
  ::std::vector<Position3D>
  Process(Position3D start, Position3D end,
          ::src::controls::ground_controls::timeline::GroundProgram ground_program);

 private:
  ::std::shared_ptr<GridStateSpace> state_space_;
  BiRRT birrt_;
};

} // namespace rrt_avoidance
} // namespace lib
