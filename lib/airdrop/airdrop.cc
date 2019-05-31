#include "airdrop.h"

namespace lib {
namespace airdrop {

Airdrop::Airdrop() : 
  _state(STANDBY),
  dropTriggered(false),
  _run(true),
  timeTriggered(-1),
  motor_pwm(0) {}

void Airdrop::getStatus(AirdropStatus & curr_status) {
  // How do we get the information from the ground station?
}

void Airdrop::Run() {
  while(_run) {
    RunIteration();
  }
}

void Airdrop::RunIteration() {
  // @TODO: Add wait function

  State next_state = _state;

  // Get the inputs from the drone
  Airdropstatus curr_status;
  getStatus(curr_status);

  // Wait times, assuming current_time in ns? Change as necessary
  double steadyTime = 5e9; // Time to steady after unlatch
  double groundTime = 5e9; //Idle time on ground
  double burnTime = 7e9; // Hot wire burn time

   double current_time =
      ::std::chrono::duration_cast<::std::chrono::nanoseconds>(
          ::std::chrono::system_clock::now().time_since_epoch())
          .count() *
      1e-9;

  switch(_state) {
    case STANDBY:
      if(curr_status.servoRelease) {
        next_state = UNLATCH_SERVO;
      } else {
        next_state = WAIT;
      }
      break;

    case UNLATCH_SERVO:
      // Unlatch the servo
      dropTriggered = true;
      next_state = WAIT;
      timeTriggered = current_time;
      break;

    case WAIT:
      // Check to see if we still need to wait
      if(current_time >= timeTriggered + waitTime) {
        next_state = MOTOR;
      } else {
        next_state = WAIT;
      }
      break;

    case MOTOR:
      motor_pwm = curr_status.motorPWM;
      if( motor_pwm > 0) { // Which way is which? assume this is down
        if(curr_status.onGround) {
          next_state = ONGROUND;
          timeTriggered = current_time;
        } else {
          next_state = MOTOR;
        }
      } else {
        if(curr_status.atTop)
          next_state = STANDBY;
        } else {
          next_state = MOTOR;
        }
      break;

    case ONGROUND:
    // Wait on the ground for a little
      if(current_time >= timeTriggered + groundTime) {
        next_state = HOTWIRE;
        timeTriggered = current_time;
      } else {
        next_state = ONGROUND;
      }
      break;

    case HOTWIRE:
      if(curr_status.hotwire) { // Start burning
        if(current_time >= timeTriggered + burnTime) {
          next_state = DONE;
        } else {
          next_state = HOTWIRE;
        }
      } else {
        next_state = HOTWIRE;
      }

    case DONE:
    // We done, stop sending info
      _run = false;
      break;
  }

  _state = next_state;
}

} // namespace airdrop
} // namespace lib