#include "lib/logger/log_writer.h"

int main() {
  ::lib::logger::LogWriter log_writer;
  select(0, nullptr, nullptr, nullptr, nullptr);
}