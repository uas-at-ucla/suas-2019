#include "src/control/loops/flight_loop.h"

#include <signal.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include "gtest/gtest.h"

#include "aos/linux_code/init.h"

#include "src/control/loops/flight_loop.q.h"

namespace spinny {
namespace control {
namespace loops {
namespace testing {

pid_t simulator_pid, io_pid, socat_pid;

void create_procs() {
  simulator_pid = fork();
  if (!simulator_pid) {
    setsid();
    const char *simulator_path = "../../../external/PX4_sitl/jmavsim";

    int null_fd = open("/dev/null", O_WRONLY);
    dup2(null_fd, 1);  // redirect stdout
    dup2(null_fd, 2);  // redirect stderr

    execl("/bin/sh", "sh", "-c", simulator_path, NULL);
  }
  ASSERT_TRUE(0 == kill(simulator_pid, 0));

  socat_pid = fork();
  if (!socat_pid) {
    setsid();
    int null_fd = open("/dev/null", O_WRONLY);
    dup2(null_fd, 1);  // redirect stdout
    dup2(null_fd, 2);  // redirect stderr

    execl("/usr/bin/socat", "socat", "pty,link=/tmp/virtualcom0,raw",
          "udp4-listen:14540", NULL);
  }
  ASSERT_TRUE(0 == kill(socat_pid, 0));

  io_pid = fork();
  if (!io_pid) {
    setsid();
    int null_fd = open("/dev/null", O_WRONLY);
    dup2(null_fd, 1);  // redirect stdout
    dup2(null_fd, 2);  // redirect stderr

    const char *io_path = "../io/io";
    execl("/bin/sh", "sh", "-c", io_path, NULL);
  }

  ASSERT_TRUE(0 == kill(io_pid, 0));
}

void kill_and_wait(pid_t pid, bool should_kill) {
  ::std::cout << "trying to kill " << pid << ::std::endl;

  kill(-1 * pid, should_kill ? SIGKILL : SIGINT);
  waitpid(-1 * pid, NULL, 0);

  ::std::cout << "KILLED " << pid << ::std::endl;
}

void quit_procs() {
  kill_and_wait(io_pid, true);
  kill_and_wait(simulator_pid, true);
  kill_and_wait(socat_pid, true);

  pid_t socat_rm_pid = fork();
  if (!socat_rm_pid) {
    execl("/bin/rm", "rm", "/tmp/virtualcom0", NULL);
  }
}

void quit_handler(int sig) {
  (void)sig;
  printf("\n\nTERMINATING TEST\n\n");

  quit_procs();

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
    ::std::cout << "\n\nTEST CASE SETUP\n\n";
    create_procs();
  }

  void TearDown() {
    ::std::cout << "\n\nTEST CASE CLEANUP\n\n";
    quit_procs();
  }

  FlightLoopQueue flight_loop_queue_;
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

  for (int i = 0; i < 1000 && !flight_loop_queue.sensors->armed; i++) {
    flight_loop_queue_.goal.MakeWithBuilder().run_mission(true).Send();

    StepLoop();
    ASSERT_TRUE(flight_loop_queue_.output.FetchLatest());
  }

  ASSERT_TRUE(flight_loop_queue.sensors->armed);

  ::std::cout << "FINAL STATE:\n";
  flight_loop_.DumpSensors();
}

TEST_F(FlightLoopTest, ArmCheck2) {
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

  for (int i = 0; i < 1000 && !flight_loop_queue.sensors->armed; i++) {
    flight_loop_queue_.goal.MakeWithBuilder().run_mission(true).Send();

    StepLoop();
    ASSERT_TRUE(flight_loop_queue_.output.FetchLatest());
  }

  ASSERT_TRUE(flight_loop_queue.sensors->armed);

  ::std::cout << "FINAL STATE:\n";
  flight_loop_.DumpSensors();
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
