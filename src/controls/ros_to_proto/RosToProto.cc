#include "RosToProto.h"

namespace src {
namespace controls {
namespace ros_to_proto {

/********** ROS TO PROTO **********/

RosToProto() {}

/**********************************/

RosToSensor() :
   global_subscriber_(
         ros_node_handle_.subscribe(kRosGlobalTopic, kRosMessageQueueSize,
                                    &RosToSensor::GlobalReceived, this)),


} // ros_to_proto
} // controls
} // src
