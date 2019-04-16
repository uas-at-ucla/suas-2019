#include "gtest/gtest.h"

#include "src/controls/messages.pb.h"

#include "serial_device.h"

namespace src {
namespace controls {
namespace ground_server {
namespace testing {

TEST(SerialDeviceTest, Creation) {
  SerialDevice<::src::controls::Sensors> serial_device("/dev/null", B9600, 0);
}

} // namespace testing
} // namespace ground_server
} // namespace controls
} // namespace src
