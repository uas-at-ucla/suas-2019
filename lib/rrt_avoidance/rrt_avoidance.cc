#include "rrt_avoidance.h"

namespace lib {
namespace rrt_avoidance {

const int kDimension = 50;

RRTAvoidance::RRTAvoidance() :
    state_space_(::std::make_shared<GridStateSpace>(kDimension, kDimension,
                                                    kDimension, kDimension)),
    birrt_(state_space_, hash, dimensions) {
  birrt_.set_asc_enabled(true);
  birrt_.set_min_iterations(4e2);
  birrt_.set_max_iterations(1e4);
  birrt_.set_goal_bias(0.1);
  birrt_.set_step_size(1.0);
  birrt_.set_max_step_size(15.0);
}

::std::vector<Position3D>
RRTAvoidance::Process(Position3D start, Position3D end,
                      ::lib::mission_manager::Obstacles obstacles) {
  if (start.latitude == end.latitude && start.longitude == end.longitude) {
    return ::std::vector<Position3D>();
  }

  birrt_.Reset();

  int rows = 2 + obstacles.static_obstacles_size();

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
    m(i, 0) = obstacles.static_obstacles(i - 2).location().latitude();
    m(i, 1) = obstacles.static_obstacles(i - 2).location().longitude();
    m(i, 2) = obstacles.static_obstacles(i - 2).cylinder_radius() * 1.3;

    double lat_diff = m(i, 0) - m(0, 0);
    double long_diff = m(i, 1) - m(0, 1);
    double dist_to_center = sqrt(lat_diff * lat_diff + long_diff * long_diff);
    double coordinate_to_meter = GetDistance2D({0, 0, 0}, {1, 0, 0});

    if (dist_to_center * coordinate_to_meter < 1.2 * m(i, 2)) {
      double multiplication_factor =
          (1.4 * m(i, 2)) / (dist_to_center * coordinate_to_meter);
      m(0, 0) = m(i, 0) - lat_diff * multiplication_factor;
      m(0, 1) = m(i, 1) - long_diff * multiplication_factor;
    }
  }

  // Shift everything so that the drone position at the origin.
  ::Eigen::MatrixXd m_trans(rows, 3);
  m_trans = m.row(0).replicate(rows, 1);
  m -= m_trans;

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

  // Scale everything to be within the obstacle grid used by RRT.
  double meter_width = m(1, 0);
  double meter_height = m(1, 1);
  double longest_dim = meter_width > meter_height ? meter_width : meter_height;

  double scale = (kDimension - 4) / longest_dim;
  m *= scale;

  // Leave a 1 unit buffer around everything to make rrt more stable.
  ::Eigen::MatrixXd m_shift(rows, 3);
  ::Eigen::MatrixXd m_shift_row(1, 3);
  m_shift_row << 2, 2, 0;
  m_shift = m_shift_row.replicate(rows, 1);
  m += m_shift;

  // Set start and end goals.
  ::Eigen::Vector2d position = {m(0, 0), m(0, 1)};
  ::Eigen::Vector2d goal = {m(1, 0), m(1, 1)};

  ::std::vector<double> obs_x, obs_y;

  // Reset obstacle grid.
  for (int y = 0; y < kDimension; y++) {
    for (int x = 0; x < kDimension; x++) {
      state_space_->obstacleGrid().obstacleAt(::Eigen::Vector2i(x, y)) = false;
    }
  }

  // Draw obstacles as circles on the obstacle grid.
  for (int i = 2; i < rows; i++) {
    int obstacle_radius = m(i, 2);
    for (int offset_x = -obstacle_radius; offset_x <= obstacle_radius;
         offset_x++) {
      int y_max = sqrt(pow(obstacle_radius, 2) - pow(offset_x, 2)) + 1;

      for (int offset_y = -y_max; offset_y <= y_max; offset_y++) {
        int x = m(i, 0) + offset_x;
        int y = m(i, 1) + offset_y;

        if (x < 0 || x >= kDimension || y < 0 || y >= kDimension) {
          continue;
        }
        state_space_->obstacleGrid().obstacleAt(::Eigen::Vector2i(x, y)) = true;

        obs_x.push_back(x);
        obs_y.push_back(y);
      }
    }
  }

  // Run the RRT.
  birrt_.set_start_state(position);
  birrt_.set_goal_state(goal);

  int num_iterations = birrt_.Run();
  ::std::vector<::Eigen::Vector2d> path = birrt_.GetPath();
  ::std::vector<::Eigen::Vector2d> smooth_path(path);
  SmoothPath<::Eigen::Vector2d>(smooth_path, *state_space_);

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

  birrt_.Reset();

  return final_path;
}

} // namespace rrt_avoidance
} // namespace lib
