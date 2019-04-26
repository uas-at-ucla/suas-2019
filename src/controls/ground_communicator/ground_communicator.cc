#include "ground_communicator.h"

namespace src {
namespace controls {
namespace ground_communicator {

GroundCommunicator::GroundCommunicator() :
  sensors_subscriber_(ros_node_handle_.subscribe(
          io::kRosSensorsTopic, io::kRosMessageQueueSize,
          &GroundCommunicator::SensorsReceived, this)),
  rfd900_("/dev/ttyUSB0", B57600, 0) {}

void GroundCommunicator::SensorsReceived(
    const ::src::controls::Sensors sensors) {
  (void) sensors;

  static int count = 0;
  if(count == 0) {
    ::src::controls::UasMessage uas_message;
    ::src::controls::Sensors *sensors_allocated = new ::src::controls::Sensors(sensors);

    uas_message.set_allocated_sensors(sensors_allocated);
    rfd900_.WritePort(uas_message);
  }

  count = (count + 1) % 2;
}

} // namespace ground_communicator
} // namespace controls
} // namespace src
