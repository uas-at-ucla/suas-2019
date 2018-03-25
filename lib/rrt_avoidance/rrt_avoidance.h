#ifndef SPINNY_LIB_RRT_AVOIDANCE_RRT_AVOIDANCE_H_
#define SPINNY_LIB_RRT_AVOIDANCE_RRT_AVOIDANCE_H_

#include "lib/rrt_avoidance/birrt.h"
#include "lib/rrt_avoidance/2dplane/2dplane.hpp"
#include "lib/rrt_avoidance/2dplane/GridStateSpace.hpp"
#include "lib/rrt_avoidance/planning/Path.hpp"
#include "lib/physics_structs/physics_structs.h"

#include "matplotlibcpp.h"

namespace lib {
namespace rrt_avoidance {

class RRTAvoidance {
 public:
  RRTAvoidance();
  ::std::vector<Position3D> Process(Position3D start, Position3D end,
              ::std::vector<Obstacle> obstacles);

 private:
  ::std::shared_ptr<GridStateSpace> state_space_;
  BiRRT birrt_;
};

}  // namespace rrt_avoidance
}  // namespace lib

#endif  // SPINNY_LIB_RRT_AVOIDANCE_RRT_AVOIDANCE_H_
