#include "alarm.h"

namespace lib {
namespace alarm {

Alarm::Alarm(double frequency) : progress_(0), loop_period_(1 / frequency) {}

void Alarm::AddAlert(AlertPeriod alert_period) { alerts_.push(alert_period); }

void Alarm::ClearAlerts() {
  while (!alerts_.empty()) {
    alerts_.pop();
  }

  progress_ = 0;
}

bool Alarm::ShouldAlarm() {
  AlertPeriod current_alert;

  bool current = true;

  do {
    // Reset progress if we go to the next alert.
    if (!current) {
      progress_ = 0;
      alerts_.pop();
    }
    current = false;

    // Do not alert if there are no alerts to process.
    if (alerts_.empty())
      return false;
    current_alert = alerts_.front();
  } while (current_alert.on + current_alert.off < progress_);

  bool should_alarm = progress_ < current_alert.on;
  progress_ += loop_period_;

  return should_alarm;
}

} // namespace alarm
} // namespace lib
