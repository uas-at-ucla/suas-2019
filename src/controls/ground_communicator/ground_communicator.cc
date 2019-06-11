#include "ground_communicator.h"

namespace src {
namespace controls {
namespace ground_communicator {

GroundCommunicator::GroundCommunicator() :
    ros_node_handle_(),
    sensors_subscriber_(
        ros_node_handle_.subscribe(kRosSensorsTopic, kRosMessageQueueSize,
                                   &GroundCommunicator::SensorsReceived, this)),
    rfd900_("/dev/ttyUSB0", B57600, 0),
    next_rfd900_write_(0) {}

void GroundCommunicator::SensorsReceived(
    const ::src::controls::Sensors sensors) {
  double current_time = ::ros::Time::now().toSec();

  if (current_time < next_rfd900_write_) {
    return;
  }
  next_rfd900_write_ = current_time + 1.0 / kRfd900SendRate;

  ::src::controls::UasMessage uas_message;
  ::src::controls::Sensors *sensors_allocated =
      new ::src::controls::Sensors(sensors);

  uas_message.set_allocated_sensors(sensors_allocated);

  bool required_fields_set = uas_message.IsInitialized();
  if (required_fields_set) {
    rfd900_.WritePort(uas_message);
  }
}

} // namespace ground_communicator
} // namespace controls
} // namespace src
