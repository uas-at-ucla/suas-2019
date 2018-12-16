#pragma once

#include "../drone_state_machine.hh"

#include <chrono>

namespace src {
namespace controls {
namespace ground_server {
namespace drone_state_machine {
namespace states {

using namespace src::controls::ground_server::drone_state_machine;

class Sleep : public BranchingDroneState {
 public:
  typedef std::chrono::steady_clock Clock;

  constexpr static BranchId NEXT = 1;

  Sleep();
  Sleep(Clock::duration sleep_duration);

  const std::vector<BranchId> ListBranches() const override;

  Clock::duration Duration() const;
  void SetDuration(Clock::duration sleep_duration);
  void SetDurationSecs(double duration_seconds);

 protected:
  void Initialize(DroneContext ctx) override;
  Result Step(DroneContext ctx) override;

 private:
  Clock::duration sleep_duration_;
  Clock::time_point start_time_;
};

} // namespace states
} // namespace drone_state_machine
} // namespace ground_server
} // namespace controls
} // namespace src
