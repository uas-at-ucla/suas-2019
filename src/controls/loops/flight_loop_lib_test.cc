#include "src/controls/loops/flight_loop.h"

#include <fcntl.h>
#include <getopt.h>
#include <signal.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include "gtest/gtest.h"

namespace src {
namespace controls {
namespace loops {
namespace testing {

bool verbose = false;

pid_t simulator_pid = 0, io_pid = 0, mavproxy_pid = 0;

void kill_and_wait(pid_t pid, bool should_kill) {
  if (verbose) {
    ::std::cout << "trying to kill " << pid << ::std::endl;
  }

  kill(-1 * pid, should_kill ? SIGKILL : SIGINT);
  waitpid(-1 * pid, NULL, 0);

  if (verbose) {
    ::std::cout << "KILLED " << pid << ::std::endl;
  }
}

void create_procs() {
  simulator_pid = fork();
  if (!simulator_pid) {
    setsid();
    const char *simulator_path = "../../../external/PX4_sitl/jmavsim";

    if (!verbose) {
      int null_fd = open("/dev/null", O_WRONLY);
      dup2(null_fd, 1); // redirect stdout
      dup2(null_fd, 2); // redirect stderr
    }

    execl("/bin/sh", "sh", "-c", simulator_path, NULL);
    exit(0);
  }
  ASSERT_TRUE(0 == kill(simulator_pid, 0));

  mavproxy_pid = fork();
  if (!mavproxy_pid) {
    setsid();

    int null_fd = open("/dev/null", O_WRONLY);
    dup2(null_fd, 1); // redirect stdout
    dup2(null_fd, 2); // redirect stderr

    execl("/usr/local/bin/mavproxy.py", "--mav20 ", "--state-basedir=/tmp/ ",
          "--master=0.0.0.0:14550 ", "--out=udp:0.0.0.0:8084 ",
          "--out=udp:0.0.0.0:8085 ", NULL);
    exit(0);
  }

  ASSERT_TRUE(0 == kill(mavproxy_pid, 0));

  usleep(1e6 / 2);

  io_pid = fork();
  if (!io_pid) {
    setsid();

    if (!verbose) {
      int null_fd = open("/dev/null", O_WRONLY);
      dup2(null_fd, 1); // redirect stdout
      dup2(null_fd, 2); // redirect stderr
    }

    const char *io_path = "../io/io";
    execl("/bin/sh", "sh", "-c", io_path, NULL);
    exit(0);
  }

  ASSERT_TRUE(0 == kill(io_pid, 0));
}

void quit_procs() {
  kill_and_wait(io_pid, true);
  kill_and_wait(simulator_pid, false);
  kill_and_wait(mavproxy_pid, true);
}

void quit_handler(int sig) {
  (void)sig;
  quit_procs();

  exit(0);
}

class FlightLoopTest : public ::testing::Test {
 protected:
  FlightLoopTest() {
    flight_loop_.SetVerbose(verbose);

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

  void SendPosition() {}

  void SetUp() { create_procs(); }

  void TearDown() { quit_procs(); }

  FlightLoop flight_loop_;
};

// TEST_F(FlightLoopTest, ArmTakeoffAndLandCheck) {
//  for (int i = 0; i < 1000; i++) {
//    flight_loop_queue_.goal.MakeWithBuilder()
//        .run_mission(false)
//        .trigger_failsafe(false)
//        .trigger_throttle_cut(false)
//        .Send();

//    StepLoop();

//    flight_loop_queue_.output.FetchLatest();
//    ASSERT_TRUE(flight_loop_queue_.output->velocity_x == 0);
//    ASSERT_TRUE(flight_loop_queue_.output->velocity_y == 0);
//    ASSERT_TRUE(flight_loop_queue_.output->velocity_z == 0);
//    ASSERT_FALSE(flight_loop_queue_.output->arm);
//    ASSERT_FALSE(flight_loop_queue_.output->takeoff);
//    ASSERT_FALSE(flight_loop_queue_.output->land);
//    ASSERT_FALSE(flight_loop_queue_.output->throttle_cut);
//  }

//  // Do arm and check if times out.
//  ::std::cout << "sending arm...\n";
//  for (int i = 0; i < 1000 && !flight_loop_queue.sensors->armed; i++) {
//    flight_loop_queue_.goal.MakeWithBuilder()
//        .run_mission(true)
//        .trigger_failsafe(false)
//        .trigger_throttle_cut(false)
//        .Send();

//    StepLoop();
//    ASSERT_TRUE(flight_loop_queue_.output.FetchLatest());
//  }
//  ASSERT_TRUE(flight_loop_queue.sensors->armed);

//  // Do takeoff and check if times out.
//  ::std::cout << "sending mission...\n";
//  for (int i = 0;
//       i < 5000 && (flight_loop_queue.sensors->relative_altitude < 2.2 ||
//                    !flight_loop_queue.sensors->armed);
//       i++) {
//    flight_loop_queue_.goal.MakeWithBuilder().run_mission(true).Send();

//    StepLoop();
//    ASSERT_TRUE(flight_loop_queue_.output.FetchLatest());
//  }
//  ASSERT_GE(flight_loop_queue.sensors->relative_altitude, 2.1);

//  ::std::cout << "flying in the air for a bit...\n";

//  // Stay in IN_AIR for a bit.
//  for (int i = 0; i < 500; i++) {
//    flight_loop_queue_.goal.MakeWithBuilder().run_mission(true).Send();

//    StepLoop();

//    ASSERT_TRUE(flight_loop_queue_.output.FetchLatest());
//    ASSERT_GE(flight_loop_queue.sensors->relative_altitude, 2.1);
//    ASSERT_TRUE(flight_loop_queue.sensors->armed);
//  }

//  ::std::cout << "end mission...\n";
//  // Do land and check if times out.
//  for (int i = 0; i < 5000 && flight_loop_queue.sensors->armed; i++) {
//    flight_loop_queue_.goal.MakeWithBuilder().run_mission(false).Send();

//    StepLoop();
//    ASSERT_TRUE(flight_loop_queue_.output.FetchLatest());
//  }
//  ASSERT_FALSE(flight_loop_queue.sensors->armed);

//  if (verbose) {
//    ::std::cout << "FINAL STATE:\n";
//    flight_loop_.DumpSensors();
//  }
//}

// TEST_F(FlightLoopTest, FailsafeCheck) {
//  flight_loop_queue_.output.FetchLatest();
//  flight_loop_queue_.sensors.FetchLatest();

//  ::std::cout << "taking off" << ::std::endl;
//  for (int i = 0;
//       (i < 10000) && (flight_loop_queue_.sensors->relative_altitude < 2.0 ||
//                       !flight_loop_queue_.sensors->armed);
//       i++) {
//    flight_loop_queue_.goal.MakeWithBuilder()
//        .run_mission(true)
//        .trigger_failsafe(false)
//        .trigger_throttle_cut(false)
//        .Send();

//    StepLoop();

//    ASSERT_TRUE(flight_loop_queue_.output.FetchLatest());

//    flight_loop_queue_.sensors.FetchLatest();
//  }

//  ASSERT_TRUE(flight_loop_queue_.sensors->armed);
//  ASSERT_GE(flight_loop_queue_.sensors->relative_altitude, 2);

//  ::std::cout << "triggering failsafe" << ::std::endl;
//  for (int i = 0; i < 10000 && flight_loop_queue.sensors->armed; i++) {
//    flight_loop_queue_.goal.MakeWithBuilder()
//        .run_mission(true)
//        .trigger_failsafe(true)
//        .trigger_throttle_cut(false)
//        .Send();

//    StepLoop();
//    ASSERT_TRUE(flight_loop_queue_.output.FetchLatest());
//  }

//  ASSERT_FALSE(flight_loop_queue.sensors->armed);
//  ::std::cout << "landed" << ::std::endl;

//  if (verbose) {
//    ::std::cout << "FINAL STATE:\n";
//    flight_loop_.DumpSensors();
//  }
//}

// TEST_F(FlightLoopTest, ThrottleCutCheck) {
//  int timeout = 0;

//  ::std::cout << "taking off" << ::std::endl;
//  do {
//    flight_loop_queue_.goal.MakeWithBuilder()
//        .run_mission(true)
//        .trigger_failsafe(false)
//        .trigger_throttle_cut(false)
//        .Send();

//    StepLoop();

//    ASSERT_TRUE(flight_loop_queue_.output.FetchLatest());
//  } while (++timeout < 10000 && flight_loop_.state() != FlightLoop::IN_AIR);

//  ASSERT_TRUE(flight_loop_queue_.sensors.FetchLatest());
//  ASSERT_TRUE(flight_loop_queue_.sensors->armed);
//  ASSERT_GE(flight_loop_queue_.sensors->relative_altitude, 2.1);

//  ::std::cout << "throttle cut" << ::std::endl;
//  for (int i = 0; i < 5000 && flight_loop_queue.sensors->relative_altitude >
//  0.1;
//       i++) {
//    flight_loop_queue_.goal.MakeWithBuilder()
//        .run_mission(true)
//        .trigger_failsafe(false)
//        .trigger_throttle_cut(true)
//        .Send();

//    StepLoop();
//    ASSERT_TRUE(flight_loop_queue_.output.FetchLatest());
//  }

//  ASSERT_LT(flight_loop_queue.sensors->relative_altitude, 0.1);
//  ::std::cout << "CRASH!" << ::std::endl;

//  if (verbose) {
//    ::std::cout << "FINAL STATE:\n";
//    flight_loop_.DumpSensors();
//  }
//}

} // namespace testing
} // namespace loops
} // namespace controls
} // namespace src

int main(int argc, char **argv) {
  signal(SIGINT, ::src::controls::loops::testing::quit_handler);
  signal(SIGTERM, ::src::controls::loops::testing::quit_handler);

  static struct option getopt_options[] = {{"verbose", no_argument, 0, 'v'},
                                           {0, 0, 0, 0}};

  while (1) {
    int opt = getopt_long(argc, argv, "i:o:sc", getopt_options, NULL);
    if (opt == -1)
      break;

    switch (opt) {
      case 'v':
        ::src::controls::loops::testing::verbose = true;
        break;
      default:
        exit(1);
        break;
    }
  }

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}