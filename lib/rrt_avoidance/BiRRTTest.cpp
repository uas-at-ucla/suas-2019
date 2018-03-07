#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <iostream>

#include "2dplane/2dplane.hpp"
#include "2dplane/GridStateSpace.hpp"
#include "BiRRT.hpp"
#include "planning/Path.hpp"
#include <stdio.h>
#include <math.h>
#include <cmath>
#include "matplotlibcpp.h"
#include "lib/physics_structs/physics_structs.h"

const int DISPLAY_MODE = 1;

using namespace std;
using namespace Eigen;

namespace RRT {

TEST(BiRRT, Instantiation) {
  BiRRT<Vector2d> biRRT(make_shared<GridStateSpace>(100, 100, 100, 100), hash,
                        dimensions);
}


  TEST(BiRRT, MatrixTest) {

    //sort out unnecessary obstacles here, given we know start and end lat, long
    int numObstacles = 1;
    int rows = 2 + numObstacles;
    ::Eigen::MatrixXd position_m(rows, 3);
    ::Eigen::MatrixXd radius_m(numObstacles, 1);

    ::std::cout.precision(10);
    
    for (int i = 0; i < numObstacles; i++)
	radius_m(i, 0) = 92;

    position_m(0,0) = 1; position_m(0,1) = 1;
    position_m(1,0) = 5; position_m(1,1) = 5;
    position_m(2,0) = 2.5; position_m(2,1) = 2.5;
    
    for (int i = 0; i < rows; i++)
      {
	if (i < 2)
	  position_m(i, 2) = 0;
	else
	  position_m(i, 2) = radius_m(i - 2, 0);
      }

    position_m -= position_m.row(0).replicate(rows, 1);
    
    double sign1, sign2;
    if (position_m(1,0) >= 0) sign1 = 1.0;
    else sign1 = -1.0;
    if (position_m(1,1) >= 0) sign2 = 1.0;
    else sign2 = -1.0;

    ::Eigen::MatrixXd rotation_m(3, 3);
    for (int i = 0; i < 3; i++)
      {
	for (int j = 0; j < 3; j++)
	  {
	    rotation_m(i, j) = 0;
	  }
      }
    rotation_m(0,0) = sign1;
    rotation_m(1,1) = sign2;
    rotation_m(2,2) = 1.0;

    position_m *= rotation_m;
    
    ::Eigen::MatrixXd meter_pos_m(rows, 3);
    for (int i = 0; i < rows; i++)
      {
	meter_pos_m(i,0) = ::spinny::GetDistance2D({0, 0, 0}, {position_m(i, 0), 0, 0});
	meter_pos_m(i,1) = ::spinny::GetDistance2D({0, 0, 0}, {0, position_m(i, 1), 0});
	meter_pos_m(i,2) = position_m(i, 2);
      }

    double dist = sqrt((meter_pos_m(1,0) - meter_pos_m(0,0)) * (meter_pos_m(1,0) - meter_pos_m(0,0)) + (meter_pos_m(1,1) - meter_pos_m(0,1)) * (meter_pos_m(1,1) - meter_pos_m(0,1)));
    double buffer = dist * 0.2;

    ::Eigen::MatrixXd buffer_m(rows, 3);
    for (int i = 0; i < rows; i++)
      {
	buffer_m(i,0) = buffer;
	buffer_m(i,1) = buffer;
	buffer_m(i,2) = 0;
      }
    meter_pos_m += buffer_m;

    double meter_width = meter_pos_m(1,0);
    double meter_height = meter_pos_m(1,1);
    double longest_dim = meter_width > meter_height ? meter_width : meter_height;
    double scale = 50 / longest_dim;

    ::Eigen::MatrixXd rrt_m(rows, 3);
    rrt_m = scale * meter_pos_m;
    std::shared_ptr<RRT::GridStateSpace> state_space = make_shared<GridStateSpace>(50, 50, 50, 50);

    Vector2d start = {rrt_m(0,0), rrt_m(0,1)}, goal = {rrt_m(1,0), rrt_m(1,1)};
    BiRRT<Vector2d> biRRT(state_space, hash, dimensions);
    state_space->obstacleGrid().obstacleAt(Vector2i(rrt_m(2,0), rrt_m(2,1))) = true;
    biRRT.setStartState(start);
    biRRT.setGoalState(goal);
    biRRT.setStepSize(1);
    biRRT.setMaxIterations(10000);
    bool success = biRRT.run();
  vector<Vector2d> path = biRRT.getPath();
  vector<Vector2d> smooth_path(path);
  RRT::SmoothPath<Vector2d>(smooth_path, *state_space);

  ::Eigen::MatrixXd rrt_out_m(smooth_path.size(), 3);
  for (int i = 0; i < smooth_path.size(); i++)
    {
      rrt_out_m(i, 0) = smooth_path[i][0];
      rrt_out_m(i, 1) = smooth_path[i][1];
      rrt_out_m(i, 2) = 0;
    }

  cout << rrt_out_m << endl;
  
  ::Eigen::MatrixXd meter_out_m(smooth_path.size(), 3);
  meter_out_m = (1 / scale) * rrt_out_m;
  
  ::Eigen::MatrixXd buffer_out_m(smooth_path.size(), 3);
  for (int i = 0; i < smooth_path.size(); i++)
      {
	buffer_out_m(i,0) = buffer;
	buffer_out_m(i,1) = buffer;
	buffer_out_m(i,2) = 0;
      }
  meter_out_m -= buffer_out_m;

  //PENDING...
  //convert to latitude and longitude
  //divide by rotation
  //transfer start's 0,0 to original position
  
  }

}  // namespace RRT
