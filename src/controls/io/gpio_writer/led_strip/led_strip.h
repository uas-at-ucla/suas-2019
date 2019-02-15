#pragma once

#include <stdint.h>

#include "ws2811.h"

namespace src {
namespace controls {
namespace io {
namespace gpio_writer {
namespace led_strip {
namespace {
static const int kNumberOfLeds = 8;
static const int kLedStripTargetFrequency = 800000;
static const int kLedStripGpioPin = 18;
static const int kLedStripDma = 5;
static const int kLedStripType = WS2811_STRIP_GBR;
} // namespace

class LedStrip {
 public:
  LedStrip();
  ~LedStrip();

  bool Render();

 private:
  void SetLed(int led, unsigned char r, unsigned char g, unsigned char b);

  ws2811_t leds_;
};

} // namespace led_strip
} // namespace gpio_writer
} // namespace io
} // namespace controls
} // namespace src
