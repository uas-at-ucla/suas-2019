#include "src/control/loops/flight_loop.h"

#include "gtest/gtest.h"

#include "aos/linux_code/init.h"

#include "src/control/loops/flight_loop.q.h"

namespace spinny {
namespace control {
namespace loops {
namespace testing {

class FlightLoopTest : public ::testing::Test {
 protected:
  FlightLoopTest()
      : flight_loop_queue_(".spinny.control.loops.flight_loop_queue",
                           0x0,
                           ".spinny.control.loops.flight_loop_queue.sensors",
                           ".spinny.control.loops.flight_loop_queue.output") {}

  void StepLoop() {
    flight_loop_.Iterate();
  }

  void CheckQueueForMessages() {
    flight_loop_queue_.output.FetchLatest();

    EXPECT_TRUE(flight_loop_queue_.output.get() != nullptr);
  }

  void SendPosition() {
    ::aos::ScopedMessagePtr<FlightLoopQueue::Sensors>
        sensors_message = flight_loop_queue_.sensors.MakeMessage();

    sensors_message.Send();
  }

 private:
  FlightLoop flight_loop_;
  FlightLoopQueue flight_loop_queue_;
};

TEST_F(FlightLoopTest, DoesNothing) {
  for(int i = 0;i < 100;i++) {
    SendPosition();
    StepLoop();
  }
}

}  // namespace testing
}  // namespace loops
}  // namespace control
}  // namespace spinny

int main(int argc, char **argv) {
  ::aos::InitCreate();
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
