#pragma once

#include <chrono>

namespace lib {
namespace trigger {

class Trigger {
 public:
  Trigger(double tolerance);
  bool Process(double trigger_time);

 private:
  double tolerance_;
  bool did_trigger_;
};

} // namespace trigger
} // namespace lib
