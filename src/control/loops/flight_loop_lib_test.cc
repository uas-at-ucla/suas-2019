#include "src/control/loops/flight_loop.h"

#include <unistd.h>
#include <signal.h>

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
      : flight_loop_queue_(".spinny.control.loops.flight_loop_queue", 0x0,
                           ".spinny.control.loops.flight_loop_queue.sensors",
                           ".spinny.control.loops.flight_loop_queue.status",
                           ".spinny.control.loops.flight_loop_queue.goal",
                           ".spinny.control.loops.flight_loop_queue.output") {

    // Change to the directory of the executable.
    char flight_loop_path[1024];
    ::readlink("/proc/self/exe", flight_loop_path,
               sizeof(flight_loop_path) - 1);
    ::std::string flight_loop_folder(flight_loop_path);
    flight_loop_folder =
        flight_loop_folder.substr(0, flight_loop_folder.find_last_of("\\/"));
    chdir(flight_loop_folder.c_str());
  }

  void StepLoop() { flight_loop_.Iterate(); }

  void CheckQueueForMessages() {
    flight_loop_queue_.output.FetchLatest();

    EXPECT_TRUE(flight_loop_queue_.output.get() != nullptr);
  }

  void SendPosition() {
    ::aos::ScopedMessagePtr<FlightLoopQueue::Sensors> sensors_message =
        flight_loop_queue_.sensors.MakeMessage();

    sensors_message.Send();
  }

  FlightLoopQueue flight_loop_queue_;

 private:
  FlightLoop flight_loop_;
};

TEST_F(FlightLoopTest, ArmCheck) {
  pid_t simulator_pid = fork();
  if(simulator_pid == 0) {
    const char *simulator_path = "../../../external/PX4_sitl/jmavsim";
    execl("/bin/sh", "sh", "-c", simulator_path, NULL);
  }
  ASSERT_TRUE(0 == kill(simulator_pid, 0));

  pid_t socat_pid = fork();
  if(socat_pid == 0) {
    execl("/usr/bin/socat", "socat", "pty,link=/tmp/virtualcom0,raw", 
        "udp4-listen:14540", NULL);
  }
  ASSERT_TRUE(0 == kill(socat_pid, 0));

  pid_t io_pid = fork();
  if(io_pid == 0) {
    const char *io_path = "../io/io";
    execl("/bin/sh", "sh", "-c", io_path, NULL);
  }
  ASSERT_TRUE(0 == kill(io_pid, 0));

  ::std::cout << "Starting flight loop." << ::std::endl;
  for(int i = 0;i < 100;i++) {
    flight_loop_queue_.goal.MakeWithBuilder().run_mission(false).Send();

    StepLoop();

    ASSERT_TRUE(flight_loop_queue_.output.FetchLatest());
    ASSERT_TRUE(flight_loop_queue_.output->velocity_x == 0);
    ASSERT_TRUE(flight_loop_queue_.output->velocity_y == 0);
    ASSERT_TRUE(flight_loop_queue_.output->velocity_z == 0);
    ASSERT_FALSE(flight_loop_queue_.output->arm);
    ASSERT_FALSE(flight_loop_queue_.output->takeoff);
    ASSERT_FALSE(flight_loop_queue_.output->land);
    ASSERT_FALSE(flight_loop_queue_.output->throttle_cut);
  }

  for(int i = 0;i < 100 && !flight_loop_queue.sensors->armed;i++) {
    flight_loop_queue_.goal.MakeWithBuilder().run_mission(true).Send();

    StepLoop();
    ASSERT_TRUE(flight_loop_queue_.output.FetchLatest());
  }

  ASSERT_TRUE(flight_loop_queue.sensors->armed);

  kill(simulator_pid, 15);
  kill(io_pid, 15);
  kill(socat_pid, 15);
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
