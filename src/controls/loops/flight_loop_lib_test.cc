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

class FlightLoopTest : public ::testing::Test {
 public:
  FlightLoopTest() {
    flight_loop_.SetVerbose(true);

    sensors_.set_time(0);
    sensors_.set_latitude(0);
    sensors_.set_longitude(0);
    sensors_.set_altitude(0);
    sensors_.set_relative_altitude(0);
    sensors_.set_heading(0);
    sensors_.set_velocity_x(0);
    sensors_.set_velocity_y(0);
    sensors_.set_velocity_z(0);
    sensors_.set_gps_ground_speed(0);
    sensors_.set_gps_satellite_count(0);
    sensors_.set_gps_eph(0);
    sensors_.set_gps_epv(0);
    sensors_.set_accelerometer_x(0);
    sensors_.set_accelerometer_y(0);
    sensors_.set_accelerometer_z(0);
    sensors_.set_gyro_x(0);
    sensors_.set_gyro_y(0);
    sensors_.set_gyro_z(0);
    sensors_.set_absolute_pressure(0);
    sensors_.set_relative_pressure(0);
    sensors_.set_pressure_altitude(0);
    sensors_.set_temperature(0);
    sensors_.set_battery_voltage(0);
    sensors_.set_battery_current(0);
    sensors_.set_armed(0);
    sensors_.set_autopilot_state(0);
    sensors_.set_last_gps(0);

    goal_.set_run_mission(false);
    goal_.set_trigger_failsafe(false);
    goal_.set_trigger_throttle_cut(false);

    goal_.set_trigger_takeoff(false);
    goal_.set_trigger_hold(false);
    goal_.set_trigger_offboard(false);
    goal_.set_trigger_rtl(false);
    goal_.set_trigger_land(false);
    goal_.set_trigger_arm(false);
    goal_.set_trigger_disarm(false);

    goal_.set_trigger_takeoff(false);
    goal_.set_trigger_hold(false);
    goal_.set_trigger_offboard(false);
    goal_.set_trigger_rtl(false);
    goal_.set_trigger_land(false);
    goal_.set_trigger_arm(false);
    goal_.set_trigger_disarm(false);
  }

  void StepLoop() {
    output_ = flight_loop_.RunIteration(sensors_, goal_);
    sensors_.set_time(sensors_.time() + 1.0 / ::src::controls::loops::kFlightLoopFrequency);
  }

  ::src::controls::Sensors &sensors() {
    return sensors_;
  }

  ::src::controls::Goal &goal() {
    return goal_;
  }

  ::src::controls::Output &output() {
    return output_;
  }

 private:
  FlightLoop flight_loop_;

  ::src::controls::Sensors sensors_;  
  ::src::controls::Goal goal_;  
  ::src::controls::Output output_;  
};

TEST_F(FlightLoopTest, Initialization) {
  for(double start = sensors().time();sensors().time() < start + 5;) {
    StepLoop();
    ASSERT_EQ(output().state(), ::src::controls::loops::State::STANDBY);
    ASSERT_EQ(output().velocity_x(), 0);
    ASSERT_EQ(output().velocity_y(), 0);
    ASSERT_EQ(output().velocity_z(), 0);

    ASSERT_FALSE(output().trigger_takeoff());
    ASSERT_FALSE(output().trigger_hold());
    ASSERT_FALSE(output().trigger_offboard());
    ASSERT_FALSE(output().trigger_rtl());
    ASSERT_FALSE(output().trigger_land());
    ASSERT_FALSE(output().trigger_arm());
    ASSERT_FALSE(output().trigger_disarm());
  }

  goal().set_trigger_arm(true);

  for(double start = sensors().time();sensors().time() < start + 5;) {
    StepLoop();
    // ASSERT_EQ(output().state(), ::src::controls::loops::State::ARMING);

    ASSERT_TRUE(output().trigger_arm());
    ASSERT_FALSE(output().trigger_takeoff());
    ASSERT_FALSE(output().trigger_hold());
    ASSERT_FALSE(output().trigger_offboard());
    ASSERT_FALSE(output().trigger_rtl());
    ASSERT_FALSE(output().trigger_land());
    ASSERT_FALSE(output().trigger_disarm());
  }
}

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

} // namespace testing
} // namespace loops
} // namespace controls
} // namespace src

int main(int argc, char **argv) {
  // static struct option getopt_options[] = {{"verbose", no_argument, 0, 'v'},
  //                                          {0, 0, 0, 0}};

  // while (1) {
  //   int opt = getopt_long(argc, argv, "i:o:sc", getopt_options, NULL);
  //   if (opt == -1)
  //     break;

  //   switch (opt) {
  //     case 'v':
  //       ::src::controls::loops::testing::verbose = true;
  //       break;
  //     default:
  //       exit(1);
  //       break;
  //   }
  // }

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
