
#include <math.h>
#include <memory>
#include <functional>
#include <rrt/2dplane/2dplane.hpp>

using namespace Eigen;
using namespace RRT;
using namespace std;

shared_ptr<Tree<Vector2d>> RRT::TreeFor2dPlane(
    shared_ptr<StateSpace<Eigen::Vector2d>> stateSpace, Vector2d goal,
    double step) {
    shared_ptr<Tree<Vector2d>> rrt =
        make_shared<Tree<Vector2d>>(stateSpace, hash, dimensions);

    rrt->setStepSize(step);

    rrt->setGoalState(goal);

    return rrt;
}
