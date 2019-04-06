#include "gtest/gtest.h"

#include "lib/alarm/alarm.h"

namespace lib {
namespace alarm {
namespace testing {

TEST(AlarmTest, AlarmRoutineTest) {
  double frequency = 1e2;
  Alarm alarm(frequency);

  alarm.AddAlert({0.5, 0.5});
  alarm.AddAlert({0.25, 0.25});
  alarm.AddAlert({0, 0.25});
  alarm.AddAlert({0.25, 0});

  double time = 0;
  for (; time < 3; time += 1 / frequency) {
    bool should_alarm = alarm.ShouldAlarm();

    EXPECT_FALSE(!(should_alarm == true) && time >= 0 && time < 0.5);
    EXPECT_FALSE(!(should_alarm == false) && time >= 0.5 && time < 1.0);
    EXPECT_FALSE(!(should_alarm == true) && time >= 1.0 && time < 1.25);
    EXPECT_FALSE(!(should_alarm == false) && time >= 1.25 && time < 1.75);
    EXPECT_FALSE(!(should_alarm == true) && time >= 1.75 && time < 2.0);
    EXPECT_FALSE(!(should_alarm == false) && time >= 2.0);
  }

  alarm.AddAlert({0.25, 0.25});
  for (; time < 10; time += 1 / frequency) {
    bool should_alarm = alarm.ShouldAlarm();

    EXPECT_FALSE(!(should_alarm == true) && time >= 3.0 && time < 3.25);
    EXPECT_FALSE(!(should_alarm == false) && time >= 3.25);
  }
}

} // namespace testing
} // namespace alarm
} // namespace lib
