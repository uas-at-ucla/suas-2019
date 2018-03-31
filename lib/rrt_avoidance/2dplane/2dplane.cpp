#include <math.h>
#include <functional>
#include <memory>
#include "lib/rrt_avoidance/2dplane/2dplane.hpp"

using namespace Eigen;
using namespace std;

namespace lib {
namespace rrt_avoidance {

shared_ptr<Tree<Vector2d>> TreeFor2dPlane(
    shared_ptr<StateSpace<Eigen::Vector2d>> stateSpace, Vector2d goal,
    double step) {
  shared_ptr<Tree<Vector2d>> rrt =
      make_shared<Tree<Vector2d>>(stateSpace, hash, dimensions);

  rrt->setStepSize(step);

  rrt->setGoalState(goal);

  return rrt;
}

}  // namespace rrt_avoidance
}  // namespace lib
