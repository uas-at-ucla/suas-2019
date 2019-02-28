#pragma once

#include <string>

#include <ros/console.h>
#include <ros/ros.h>

namespace src {
namespace controls {
namespace ros_to_proto {
namespace {
   static const int kRosMessageQueueSize = 1;
   static const ::std::string kRosGlobalTopic = "/mavros/global_position/global";
   static const ::std::string kRosRelAltTopic = "/mavros/global_position/rel_alt"; 
   static const ::std::string kRosHeadingTopic = "/mavros/global_position/compass_hdg";
   static const ::std::string kRosVelocityTopic = "/mavros/local_position/velocity_local";
   static const ::std::string kRosAccelTopic = "/mavros/imu/data";
} // namespace

// RosToProto base class
class RosToProto {
   public:
      RosToProto();

   protected:
      ::ros::NodeHandle ros_node_handle_;
};

// Sensors sub class
class RosToSensor public RosToProto {
   public:
      RosToSensor();

   private:
      // Private callback functions
      void GlobalReceived(const ::src::mavros_msgs::GlobalPositionTarget global_position);
      void AltitudeReceived(const ::src::mavros_msgs::Altitude altitude);
      void HeadingReceived(const ::src::mavros_msgs::VFR_HUD heading);

      // Ros subscribers
      ::ros::Subscriber global_subscriber_;
      ::ros::Subscriber altitude_subscriber_;
      ::ros::Subscriber heading_subscriber_;
      ::ros::Subscriber velocity_subscriber_;
      ::ros::Subscriber accel_subscriber_;
};

} // ros_to_proto
} // controls
} // src
