#pragma once

#include <queue>
#include <vector>

namespace lib {
namespace alarm {

struct AlertPeriod {
  double on;
  double off;
};

class Alarm {
 public:
  Alarm(double frequency);
  void AddAlert(AlertPeriod alert_period);
  bool ShouldAlarm();
  void ClearAlerts();

 private:
  ::std::queue<AlertPeriod> alerts_;
  double progress_;
  double loop_period_;
};

} // namespace alarm
} // namespace lib
