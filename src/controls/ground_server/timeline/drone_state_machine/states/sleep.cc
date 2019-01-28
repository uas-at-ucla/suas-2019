#include "sleep.hh"

namespace src {
namespace controls {
namespace ground_server {
namespace drone_state_machine {
namespace states {

constexpr Sleep::Clock::duration DEFAULT_DURATION =
    std::chrono::milliseconds(1000);

Sleep::Sleep() : Sleep(DEFAULT_DURATION) {}

Sleep::Sleep(Clock::duration sleep_duration) :
    BranchingDroneState("Sleep"),
    sleep_duration_(sleep_duration) {}

const std::vector<BranchId> Sleep::ListBranches() const { return {NEXT}; }

void Sleep::SetDuration(Clock::duration sleep_duration) {
  sleep_duration_ = sleep_duration;
}

void Sleep::SetDurationSecs(double duration_secs) {
  using namespace std::chrono;
  auto duration_s = duration<double>(duration_secs);
  Clock::duration sleep_duration = duration_cast<nanoseconds>(duration_s);
  SetDuration(sleep_duration);
}

void Sleep::Initialize(DroneContext ctx) {
  (void)ctx;
  start_time_ = Clock::now();
}

Result Sleep::Step(DroneContext ctx) {
  (void)ctx;
  auto now = Clock::now();
  if (now >= start_time_ + sleep_duration_) {
    return Branch(NEXT);
  }
  return result::YIELD;
}

} // namespace states
} // namespace drone_state_machine
} // namespace ground_server
} // namespace controls
} // namespace src