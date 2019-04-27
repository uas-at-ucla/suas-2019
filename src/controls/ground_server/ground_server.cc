#include "ground_server.h"

namespace src {
namespace controls {
namespace ground_server {

// - Handles the compiler
// - Receives data from UGV
GroundServer::GroundServer() {} // namespace ground_server

void GroundServer::UgvSensorsReceived(
    const ::src::controls::UgvSensors ugv_sensors) {
  (void)ugv_sensors;
}
void GroundServer::DroneSensorsReceived(
    const ::src::controls::Sensors drone_sensors) {
  (void)drone_sensors;
}

} // namespace ground_server
} // namespace controls
} // namespace src
