#include "ground_communicator.h"

namespace src {
namespace controls {
namespace ground_communicator {

GroundCommunicator::GroundCommunicator() :
    sensors_subscriber_(ros_node_handle_.subscribe(
        io::kRosSensorsTopic, io::kRosMessageQueueSize,
        &GroundCommunicator::SensorsReceived, this)),
    proto_sender_("tcp://127.0.0.1:6005"),
    rfd900_("/dev/ttyUSB0", B57600, 0) {

    proto_sender_.Connect();
}

void GroundCommunicator::SensorsReceived(
    const ::src::controls::Sensors sensors) {

  ::src::controls::UasMessage uas_message;
  ::src::controls::Sensors *sensors_allocated =
      new ::src::controls::Sensors(sensors);

  uas_message.set_allocated_sensors(sensors_allocated);

  bool required_fields_set = uas_message.IsInitialized();

  static int count = 0;
  if (count == 0 && required_fields_set) {
    rfd900_.WritePort(uas_message);
  }

  count = (count + 1) % 2; // 50/2 = 25 Hz

  // Send the message
  if (required_fields_set) {
    proto_sender_.Send(uas_message);
  }
}

} // namespace ground_communicator
} // namespace controls
} // namespace src
