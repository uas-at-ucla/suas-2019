#include "deployment.h"

namespace lib {
namespace deployment {

Deployment::Deployment(int loop_frequency) :
    loop_frequency_(loop_frequency),
    state_(LATCHED),
    timer_(0) {}

void Deployment::RunIteration(struct Input &input, struct Output &output) {
  // Set default state and outputs.
  State next_state = state_;
  output.motor = kDefaultMotorOutput;
  output.latch = kDefaultLatchOutput;
  output.hotwire = kDefaultHotwireOutput;

  // Handle the current state.
  switch (state_) {
    case LATCHED:
      // Check whether deployment should be initialized.
      if (input.direction > 0) {
        // Wait a certain amount of time before actually unlatching.
        if (TimerComplete(kWaitTimeBeforeUnlatch)) {
          next_state = UNLATCHING;
          break;
        }

        TickTimer();
        break;
      }

      // If deployment is no longer being triggered, reset the timer.
      ResetTimer();
      break;

    case UNLATCHING:
      // Unlatch the servo.
      output.latch = false;

      if (input.direction > 0) {
        // Wait a certain amount of time before turning on the motor.
        if (TimerComplete(kWaitTimeAfterUnlatch)) {
          next_state = MOTOR_ASSISTED_DEPLOYMENT;
          break;
        }

        TickTimer();
        break;
      }

      // If deployment is no longer being triggered, reset the timer.
      ResetTimer();
      break;

    case MOTOR_ASSISTED_DEPLOYMENT:
      // Keep the servo unlatched.
      output.latch = false;

      // If externally triggered to cut the line, change to that state.
      if (input.cut) {
        next_state = CUT_LINE;
        break;
      }

      // Write forwards or backwards motor output, if told to do so.
      if (input.direction != 0) {
        output.motor =
            (input.direction > 0) ? kMotorDeployPower : -kMotorDeployPower;
      }

      break;

    case CUT_LINE:
      // Keep the servo unlatched.
      output.latch = false;

      // Return to motor power if the hotwire is not being commanded to turn on.
      if (!input.cut) {
        next_state = MOTOR_ASSISTED_DEPLOYMENT;
        break;
      }

      // Write out hotwire.
      output.hotwire = true;
  }

  // Handle any changes in state.
  if (state_ != next_state) {
    ROS_INFO("Deployment state changed: %d -> %d", state_, next_state);
    ResetTimer();
    state_ = next_state;
  }
}

void Deployment::TickTimer() { timer_ += 1.0 / loop_frequency_; }

bool Deployment::TimerComplete(double goal_time) { return timer_ > goal_time; }

void Deployment::ResetTimer() { timer_ = 0; }

} // namespace deployment
} // namespace lib