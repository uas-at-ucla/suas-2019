#pragma once

#include <chrono>

namespace lib {
namespace airdrop {
namespace {
  static const ::std::string kRosStateTopic = "/uasatucla/airdrop/state";
  static const ::std::string kRosStatusTopic = "/uasatucla/airdrop/status";
} // Namespace

struct AirdropStatus {
  int direction;
  bool servoRelease;
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

  void rosSubscriber(::std::string action);

  private:
    atomic State _state; // State of the state machine
    bool _run; // Should we run the iteration
    bool dropTriggered; // Drop has been triggered
    double timeTriggered; // Time of the trigger
    double motor_pwm; // Value of the motor input
    bool onGround; // Has the UGV made it to the ground?

    void Run();
    void RunIteration();

    // Ros
    ::ros::NodeHandle ros_node_handle_;
    ::ros::Subscriber state_subscriber_;
    ::ros::Publisher status_publisher_;
};

} // namespace airdrop
} // namespace lib
