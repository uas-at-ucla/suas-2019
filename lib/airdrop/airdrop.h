#pragma once

#include <chrono>

namespace lib {
namespace airdrop {

struct AirdropStatus {
  double motorPWM;
  bool servoRelease;
  bool onGround;
  bool hotwire;
  bool atTop;
};

class Airdrop {
  public:
  Deployment();
  void getStatus(AirdropStatus & curr_status);

  enum State {
    STANDBY = 0,
    UNLATCH_SERVO = 1,
    WAIT = 2,
    MOTOR = 3,
    ONGROUND = 4,
    HOTWIRE = 5,
    COMPLETED = 6
  }


  private:
  State _state; // State of the state machine
  bool _run; // Should we run the iteration
  bool dropTriggered; // Drop has been triggered
  double timeTriggered; // Time of the trigger
  double motor_pwm; // Value of the motor input
  bool onGround; // Has the UGV made it to the ground?

  void Run();
  void RunIteration();
};

} // namespace airdrop
} // namespace lib
