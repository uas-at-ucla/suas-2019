#include "airdrop.h"

namespace lib {
namespace airdrop {

Airdrop::Airdrop() : 
  _state(STANDBY),
  dropTriggered(false),
  _run(true),
  timeTriggered(-1),
  motor_pwm(0),
  ros_node_handle_(),
  state_subscriber_(ros_node_handle_.subscribe(
    kRosStateTopic, ::src::controls::io::kRosMessageQueueSize,
    &Airdrop::rosSubscriber, this)),
  status_publisher_(ros_node_handle_.advertise<
    ::std::string>(
      kRosStatusTopic, ::src::controls::io::kRosMessageQueueSize))
    ))
  )){}

void Airdrop::rosSubscriber(::std::string action) {
  if (action == "RELEASE") {
    
  } else if (action == "UGV_LANDED") {

  }
}

void Airdrop::RunIteration(struct Airdropstatus & curr_status, double &motorPwm, bool &servoRelease, ) {
  // @TODO: Add wait function

  State next_state = _state;

  // Get the inputs from the drone
  struct Airdropstatus curr_status;

  // Wait times, assuming current_time in ns? Change as necessary
  const double steadyTime = 5e9; // Time to steady after unlatch
  const double groundTime = 5e9; //Idle time on ground
  const double burnTime = 7e9; // Hot wire burn time

   double current_time =
      ::std::chrono::duration_cast<::std::chrono::nanoseconds>(
          ::std::chrono::system_clock::now().time_since_epoch())
          .count() *
      1e-9;

  switch(_state) {
    case STANDBY:
      if(servoRelease) {
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
        if(current_time >= timeTriggered + burnTime) {
          next_state = DONE;
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