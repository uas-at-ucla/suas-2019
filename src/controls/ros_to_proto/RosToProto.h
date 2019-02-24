#pragma once

#include <string>

#include <ros/console.h>
#include <ros/ros.h>

namespace src {
namespace controls {
namespace ros_to_proto {
namespace {
   static const ::std::string kRosGlobalTopic = "/mavros/global_position/global";
   static const ::std::string kRosRelAltTopic = "/mavros/global_position/rel_alt"; 
   static const ::std::string kRosHeadingTopic = "/mavros/global_position/compass_hdg";
   static const ::std::string kRosVelocityTopic = "/mavros/local_position/velocity_local";
   static const ::std::string kRosAccelTopic = "/mavros/imu/data";

}

} // ros_to_proto
} // controls
} // src
