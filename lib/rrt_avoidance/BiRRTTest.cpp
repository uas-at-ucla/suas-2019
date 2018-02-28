#include <gtest/gtest.h>
#include <rrt/2dplane/2dplane.hpp>
#include <rrt/2dplane/GridStateSpace.hpp>
#include <rrt/BiRRT.hpp>

using namespace std;
using namespace Eigen;

namespace RRT {

TEST(BiRRT, Instantiation) {
    BiRRT<Vector2d> biRRT(make_shared<GridStateSpace>(50, 50, 50, 50), hash,
                          dimensions);
}

TEST(BiRRT, getPath) {
    Vector2d start = {1, 1}, goal = {30, 30};

    BiRRT<Vector2d> biRRT(make_shared<GridStateSpace>(50, 50, 50, 50), hash,
                          dimensions);
    biRRT.setStartState(start);
    biRRT.setGoalState(goal);
    biRRT.setStepSize(1);
    biRRT.setMaxIterations(10000);

    bool success = biRRT.run();
    ASSERT_TRUE(success);

    vector<Vector2d> path = biRRT.getPath();

    // path should contain at least two points (start and end)
    ASSERT_GE(path.size(), 2);

    // The given start and goal points should be the first and last points of
    // the path, respectively.
    EXPECT_EQ(start, path.front());
    EXPECT_EQ(goal, path.back());
}

TEST(BiRRT, multipleRuns) {
    Vector2d start = {1, 1}, goal = {30, 30};

    BiRRT<Vector2d> biRRT(make_shared<GridStateSpace>(50, 50, 50, 50), hash,
                          dimensions);
    biRRT.setStartState(start);
    biRRT.setGoalState(goal);
    biRRT.setStepSize(1);
    biRRT.setMaxIterations(10000);

    for (int i = 0; i < 50; i++) {
        bool success = biRRT.run();
        ASSERT_TRUE(success);
        biRRT.reset();
    }
}

}  // namespace RRT
