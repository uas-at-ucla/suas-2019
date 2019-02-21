#pragma once

#include <cmath>
#include <iostream>
#include <ros/ros.h>
#include <stdint.h>

#include "ws2811.h"

#include "lib/phased_loop/phased_loop.h"

namespace src {
namespace controls {
namespace io {
namespace gpio_writer {
namespace led_strip {
namespace {
static const int kNumberOfLeds = 10;
static const int kLedStripTargetFrequency = 800000;
static const int kLedStripGpioPin = 10;
static const int kLedStripDma = 5;
static const int kLedStripType = WS2811_STRIP_GBR;
static constexpr double kDisarmedBlinkFrequency = 0.5;
static constexpr double kBatteryBlinkFrequency = 1.5;
} // namespace

class LedStrip {
 public:
  LedStrip();
  ~LedStrip();

  bool Render();

  void set_battery_percentage(float battery_percentage) {
    battery_percentage_ = battery_percentage;
  }

  void set_armed(bool armed) { armed_ = armed; }

 private:
  void SetLed(int led, unsigned char r, unsigned char g, unsigned char b);

  ws2811_led_t *led_pixels_;
  ws2811_channel_t channel_0_;
  ws2811_channel_t channel_1_;
  ws2811_t leds_;

  float battery_percentage_;
  bool armed_;
};

} // namespace led_strip
} // namespace gpio_writer
} // namespace io
} // namespace controls
} // namespace src
