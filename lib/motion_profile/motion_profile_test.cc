#include "motion_profile.h"

#include "gtest/gtest.h"

namespace lib {
namespace motion_profile {
namespace testing {

// TEST(MotionProfileTest, SimpleProfile) {
//  MotionProfile motion_profile(5.0, 5.0, 1.0 / 100);

//  Position3D drone_position{0, 0, 0};

//  ::std::vector<Position3D> waypoints;

//  int print_interval = 10;
//  for (int i = 0; i < 1e6; i++) {
//    if(i % print_interval == 0) {
////    ::std::cout << "Iteration " << i << ::std::endl;
//    }

//    if (i == 0) {
//      waypoints.clear();
//      waypoints.push_back({0.001, 0.001, 0});
//      waypoints.push_back({0.001, -0.001, 0});
//      waypoints.push_back({-0.001, 0.001, 0});
//    }

//    ::Eigen::Vector3d new_velocity =
//        motion_profile.Calculate(drone_position, waypoints);

//    if(i % print_interval == 0) {
////    ::std::cout << new_velocity.x() << "," << new_velocity.y() <<
///::std::endl;
////    ::std::cout << "new_velocity: " << ::std::endl
////                << new_velocity << ::std::endl;
//    }
//    new_velocity /= 100;

//    drone_position.latitude +=
//        new_velocity.x() / GetDistance2D({0, 0, 0}, {1, 0, 0});
//    drone_position.longitude +=
//        new_velocity.y() / GetDistance2D({0, 0, 0}, {0, 1, 0});

//    if(waypoints.size() > 0 && GetDistance3D(drone_position, waypoints.at(0))
//    < 10) {
//      waypoints.erase(waypoints.begin());
//    }

//    if(i % print_interval == 0) {
//      ::std::cout << i / 100.0 << "," << drone_position.latitude << "," <<
//      drone_position.longitude << ::std::endl;
////    ::std::cout << "lat: " << drone_position.latitude
////                << " lng: " << drone_position.longitude << ::std::endl;
//    }
//  }
//}

} // namespace testing
} // namespace motion_profile
} // namespace lib
