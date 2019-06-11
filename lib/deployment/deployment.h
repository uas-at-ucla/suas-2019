#pragma once

#include <ros/console.h>

namespace lib {
namespace deployment {
namespace {
static constexpr double kWaitTimeBeforeUnlatch = 0;
static constexpr double kWaitTimeAfterUnlatch = 3;
static constexpr double kWaitTimeAfterHitGround = 3;
static constexpr double kMaxHotwireTime = 10;

static constexpr double kDefaultMotorOutput = 0;
static constexpr bool kDefaultLatchOutput = true;
static constexpr bool kDefaultHotwireOutput = false;

static constexpr double kMotorDeployPower = 0.6;
} // namespace

struct Input {
  bool latch;
  int direction;
  bool cut;
  bool cancel;
};

struct Output {
  double motor;
  bool latch;
  bool hotwire;
  bool end_drop;
};

enum State {
  LATCHED = 0,
  UNLATCHING = 1,
  MOTOR_ASSISTED_DEPLOYMENT = 2,
  CUT_LINE = 3,
  END_DROP = 4,
};

class Deployment {
 public:
  Deployment(int frequency);
  void RunIteration(struct Input &input, struct Output &output);

 private:
  void TickTimer();
  bool TimerComplete(double goal_time);
  void ResetTimer();

  int loop_frequency_;
  State state_;
  double timer_;
};

} // namespace deployment
} // namespace lib
