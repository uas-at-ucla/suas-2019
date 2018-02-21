#include "src/control/loops/flight_loop.h"

#include <signal.h>
#include <unistd.h>

#include "gtest/gtest.h"

#include "aos/linux_code/init.h"

#include "src/control/loops/flight_loop.q.h"

namespace spinny {
namespace control {
namespace loops {
namespace testing {

pid_t simulator_pid, io_pid, socat_pid;

void create_procs() {
  pid_t simulator_pid = fork();
  if (simulator_pid == 0) {
    const char *simulator_path = "../../../external/PX4_sitl/jmavsim";
    execl("/bin/sh", "sh", "-c", simulator_path, NULL);
  }
  ASSERT_TRUE(0 == kill(simulator_pid, 0));

  pid_t socat_pid = fork();
  if (socat_pid == 0) {
    execl("/usr/bin/socat", "socat", "pty,link=/tmp/virtualcom0,raw",
          "udp4-listen:14540", NULL);
  }
  ASSERT_TRUE(0 == kill(socat_pid, 0));

  pid_t io_pid = fork();
  if (io_pid == 0) {
    const char *io_path = "../io/io";
    execl("/bin/sh", "sh", "-c", io_path, NULL);
  }
  ASSERT_TRUE(0 == kill(io_pid, 0));
}

void kill_and_wait(pid_t pid) {
  int status;

  while (waitpid(pid, &status, WNOHANG)) {
    kill(simulator_pid, 2);
    ::std::cout << "\n\n\nkilled " << status << "\n\n\n" << ::std::endl;
  }
}

void quit_procs(bool socat) {
  kill_and_wait(simulator_pid);
  kill_and_wait(io_pid);

  if(socat) {
    kill_and_wait(socat_pid);
  }
}

void quit_handler(int sig) {
  (void)sig;
  printf("\n\nTERMINATING TEST\n\n");

  quit_procs(true);

  exit(0);
}

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
  
  void SetUp() {
    create_procs();
  }

  void TearDown() {
    quit_procs(false);
    usleep(1e6 * 2);
  }

  FlightLoopQueue flight_loop_queue_;

 private:
  FlightLoop flight_loop_;
};

TEST_F(FlightLoopTest, ArmCheck) {
  ::std::cout << "Starting flight loop." << ::std::endl;
  for (int i = 0; i < 100; i++) {
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

  for (int i = 0; i < 100 && !flight_loop_queue.sensors->armed; i++) {
    flight_loop_queue_.goal.MakeWithBuilder().run_mission(true).Send();

    StepLoop();
    ASSERT_TRUE(flight_loop_queue_.output.FetchLatest());
  }

  ASSERT_TRUE(flight_loop_queue.sensors->armed);

  usleep(1e6 * 2);
}

TEST_F(FlightLoopTest, FailsafeCheck) {
  ::std::cout << "Starting flight loop." << ::std::endl;

  flight_loop_queue_.goal.FetchLatest();
  flight_loop_queue_.output.FetchLatest();

  for (int i = 0; i < 100; i++) {
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

  for (int i = 0; i < 5000 && flight_loop_queue.sensors->relative_altitude < 2; i++) {
    flight_loop_queue_.goal.MakeWithBuilder().run_mission(true).Send();

    StepLoop();
    ASSERT_TRUE(flight_loop_queue_.output.FetchLatest());
  }

  ASSERT_TRUE(flight_loop_queue.sensors->armed);

  for (int i = 0; i < 5000 && flight_loop_queue.sensors->armed; i++) {
    flight_loop_queue_.goal.MakeWithBuilder()
        .run_mission(true)
        .trigger_failsafe(true)
        .Send();

    StepLoop();
    ASSERT_TRUE(flight_loop_queue_.output.FetchLatest());
  }

  ASSERT_FALSE(flight_loop_queue.sensors->armed);
}

}  // namespace testing
}  // namespace loops
}  // namespace control
}  // namespace spinny

int main(int argc, char **argv) {
  signal(SIGINT, ::spinny::control::loops::testing::quit_handler);
  signal(SIGKILL, ::spinny::control::loops::testing::quit_handler);
  signal(SIGTERM, ::spinny::control::loops::testing::quit_handler);
  signal(SIGQUIT, ::spinny::control::loops::testing::quit_handler);
  signal(SIGHUP, ::spinny::control::loops::testing::quit_handler);
  signal(SIGTSTP, ::spinny::control::loops::testing::quit_handler);
  signal(SIGSTOP, ::spinny::control::loops::testing::quit_handler);
  signal(SIGTTOU, ::spinny::control::loops::testing::quit_handler);
  signal(SIGTTIN, ::spinny::control::loops::testing::quit_handler);
  signal(SIGABRT, ::spinny::control::loops::testing::quit_handler);

  ::aos::InitCreate();
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
