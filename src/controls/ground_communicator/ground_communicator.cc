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
    rfd900_.WritePort("test");
  }

  count = (count + 1) % 5;
}

} // namespace ground_communicator
} // namespace controls
} // namespace src
