#ifndef LIB_ALARM_ALARM_H_
#define LIB_ALARM_ALARM_H_

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

 private:
  ::std::queue<AlertPeriod> alerts_;
  double progress_;
  double loop_period_;
};

}  // namespace alarm
}  // namespace lib

#endif  // LIB_ALARM_ALARM_H_
