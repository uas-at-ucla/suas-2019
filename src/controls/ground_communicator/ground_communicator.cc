#include "ground_communicator.h"

namespace src {
namespace controls {
namespace ground_communicator {

// - Handles the compiler
// - Receives data from UGV
GroundCommunicator::GroundCommunicator() {} // namespace ground_server

void GroundCommunicator::UgvSensorsReceived(
    const ::src::controls::UgvSensors ugv_sensors) {
  (void)ugv_sensors;
}
void GroundCommunicator::DroneSensorsReceived(
    const ::src::controls::Sensors drone_sensors) {
  (void)drone_sensors;
}

} // namespace ground_server
} // namespace controls
} // namespace src
