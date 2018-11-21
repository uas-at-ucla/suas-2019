#pragma once

namespace src {
namespace controls {
namespace ground_server {
namespace timeline {
namespace simulator {

class Simulator {
 public:
  Simulator();

 private:
  ::Eigen::Vector3d mock_drone_position_;
};

} // namespace simulator
} // namespace timeline
} // namespace ground_server
} // namespace controls
} // namespace src
