#include "rrt_avoidance.h"

namespace lib {
namespace rrt_avoidance {

::std::vector<Position3D> RRTAvoidance::Process(
    Position3D start, Position3D end, ::std::vector<Obstacle> obstacles) {
  int rows = 2 + obstacles.size();

  ::Eigen::MatrixXd m(rows, 3);

  //::std::cout.precision(10);

  // Load initial start/end and obstacle values.
  m(0, 0) = start.latitude;
  m(0, 1) = start.longitude;
  m(0, 2) = 0;
  m(1, 0) = end.latitude;
  m(1, 1) = end.longitude;
  m(1, 2) = 0;
  for (int i = 2; i < rows; i++) {
    m(i, 0) = obstacles[i].position.latitude;
    m(i, 1) = obstacles[i].position.longitude;
    m(i, 2) = obstacles[i].radius;
  }
  //::std::cout << m << ::std::endl;

  // Shift everything so that the drone position at the origin.
  ::Eigen::MatrixXd m_trans(rows, 3);
  m_trans = m.row(0).replicate(rows, 1);
  m -= m_trans;

  //::std::cout << m << ::std::endl;

  // Make the start->end points exist in the first quadrant.
  ::Eigen::MatrixXd reflection_m = ::Eigen::MatrixXd::Identity(3, 3);
  reflection_m(0, 0) = (m(1, 0) >= 0) ? 1 : -1;
  reflection_m(1, 1) = (m(1, 1) >= 0) ? 1 : -1;
  m *= reflection_m;

  // Convert coordinate components to meters.
  double coordinate_to_meter = GetDistance2D({0, 0, 0}, {1, 0, 0});

  for (int i = 0; i < rows; i++) {
    m(i, 0) = m(i, 0) * coordinate_to_meter;
    m(i, 1) = m(i, 1) * coordinate_to_meter;
  }

  // Scale everything to be within the 0 -> 50 obstacle grid used by RRT.
  const int kDimension = 50;
  double meter_width = m(1, 0);
  double meter_height = m(1, 1);
  double longest_dim = meter_width > meter_height ? meter_width : meter_height;

  double scale = (kDimension - 2) / longest_dim;
  m *= scale;

  // Leave a 1 unit buffer around everything to make rrt more stable.
  ::Eigen::MatrixXd m_shift(rows, 3);
  ::Eigen::MatrixXd m_shift_row(1, 3);
  m_shift_row << 1, 1, 0;
  m_shift = m_shift_row.replicate(rows, 1);
  m += m_shift;

  std::shared_ptr<GridStateSpace> state_space =
      ::std::make_shared<GridStateSpace>(kDimension, kDimension, kDimension,
                                         kDimension);

  // Set start and end goals.
  ::Eigen::Vector2d position = {m(0, 0), m(0, 1)};
  ::Eigen::Vector2d goal = {m(1, 0), m(1, 1)};

  BiRRT biRRT(state_space, hash, dimensions);
  ::std::vector<double> obs_x, obs_y;

  // Draw obstacles as circles on the obstacle grid.
  for (int i = 2; i < rows; i++) {
    int obstacle_radius = m(i, 2);
    for (int offset_x = -obstacle_radius; offset_x <= obstacle_radius;
         offset_x++) {
      int y_max = sqrt(pow(obstacle_radius, 2) - pow(offset_x, 2)) + 1;

      for (int offset_y = -y_max; offset_y <= y_max; offset_y++) {
        int x = m(i, 0) + offset_x;
        int y = m(i, 1) + offset_y;
        if (x < 0 || x >= kDimension | y < 0 || y >= kDimension) {
          continue;
        }

        state_space->obstacleGrid().obstacleAt(::Eigen::Vector2i(x, y)) = true;

        obs_x.push_back(x);
        obs_y.push_back(y);
      }
    }
  }

  // Run the RRT.
  biRRT.set_start_state(position);
  biRRT.set_goal_state(goal);
  biRRT.set_step_size(1);
  biRRT.set_max_iterations(10000);
  bool success = biRRT.Run();
  ::std::vector<::Eigen::Vector2d> path = biRRT.GetPath();
  ::std::vector<::Eigen::Vector2d> smooth_path(path);
  SmoothPath<::Eigen::Vector2d>(smooth_path, *state_space);

  ::Eigen::MatrixXd m_rrt_path(smooth_path.size(), 2);

  ::std::vector<double> smooth_x, smooth_y;
  for (size_t i = 0; i < smooth_path.size(); i++) {
    m_rrt_path(i, 0) = smooth_path[i][0];
    m_rrt_path(i, 1) = smooth_path[i][1];

    smooth_x.push_back(smooth_path[i][0]);
    smooth_y.push_back(smooth_path[i][1]);
  }

  //::std::cout << m_rrt_path << ::std::endl;

  // Apply inverse operations to get back to original coordinates.
  m_rrt_path -= m_shift.block<1, 2>(0, 0).replicate(smooth_path.size(), 1);
  m_rrt_path /= scale;
  m_rrt_path /= coordinate_to_meter;
  m_rrt_path *= reflection_m.block<2, 2>(0, 0);
  m_rrt_path += m_trans.block<1, 2>(0, 0).replicate(smooth_path.size(), 1);

  ::std::vector<Position3D> final_path;
  for (size_t i = 0; i < smooth_path.size(); i++) {
    Position3D waypoint = {m_rrt_path(i, 0), m_rrt_path(i, 1), 0};
    final_path.push_back(waypoint);
  }

  biRRT.Reset();

  return final_path;
}

}  // namespace rrt_avoidance
}  // namespace lib
