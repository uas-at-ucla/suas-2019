#include "led_strip.h"

namespace src {
namespace controls {
namespace io {
namespace gpio_writer {
namespace led_strip {

LedStrip::LedStrip() : battery_percentage_(0.0), armed_(false) {
  led_pixels_ =
      static_cast<ws2811_led_t *>(malloc(sizeof(ws2811_led_t) * kNumberOfLeds));

  channel_0_ = {
      kLedStripGpioPin, // GPIO number
      0,                // Invert output signal
      kNumberOfLeds,    // Number of LEDs, 0 if channel is unused.
      kLedStripType,    // Strip color layout
      led_pixels_,      // LED buffer
      255,              // Brightness value
      0,                // White shift value
      0,                // Red shift value
      0,                // Green shift value
      0,                // Blue shift value
      nullptr           // Gamma correction
  };

  channel_1_ = {
      0,             // GPIO number
      0,             // Invert output signal
      0,             // Number of LEDs, 0 if channel is unused.
      kLedStripType, // Strip color layout
      nullptr,       // LED buffer
      255,           // Brightness value
      0,             // White shift value
      0,             // Red shift value
      0,             // Green shift value
      0,             // Blue shift value
      nullptr        // Gamma correction
  };

  leds_ = {
      0,                        // Render wait time
      nullptr,                  // Device
      nullptr,                  // Raspi hardware information
      kLedStripTargetFrequency, // Required output frequency
      kLedStripDma,             // DMA number
      {channel_0_, channel_1_}  // Channels
  };

#ifdef UAS_AT_UCLA_DEPLOYMENT
  ws2811_return_t ret = ws2811_init(&leds_);

  if (ret != WS2811_SUCCESS) {
    return;
  }
#endif
}

LedStrip::~LedStrip() {
  delete led_pixels_;

#ifdef UAS_AT_UCLA_DEPLOYMENT
  ws2811_fini(&leds_);
#endif
}

bool LedStrip::Render() {
  // Render the arming state indicator.
  {
    bool should_blink =
        !armed_ && ::std::fmod(::lib::phased_loop::GetCurrentTime(),
                               1.0 / kDisarmedBlinkFrequency) <
                       1.0 / (2 * kDisarmedBlinkFrequency);

    for (int i = 0; i < 5; i++) {
      if (armed_) {
        SetLed(i, 255, 0, 0);
      } else if (should_blink) {
        SetLed(i, 0, 255, 0);
      } else {
        SetLed(i, 0, 0, 0);
      }
    }
  }

  // Render the battery level indicator.
  {
    int solid = ::std::ceil((battery_percentage_ - 0.1) / 0.2);
    bool blinky = ::std::fmod(battery_percentage_, 0.2) < 0.1;

    for (int i = 0; i < 5; i++) {
      bool should_blink = i == solid && blinky &&
                          ::std::fmod(::lib::phased_loop::GetCurrentTime(),
                                      1.0 / kBatteryBlinkFrequency) <
                              1.0 / (2 * kBatteryBlinkFrequency);
      if (i < solid || should_blink) {
        // LED should be solid.
        SetLed(9 - i, 0, 255, 0);
      } else {
        // LED should be off.
        SetLed(9 - i, 0, 0, 0);
      }
    }
  }

#ifdef UAS_AT_UCLA_DEPLOYMENT
  ws2811_return_t ret = ws2811_render(&leds_);

  if (ret != WS2811_SUCCESS) {
    return false;
  }
  // #else
  // ::std::string led_string;
  // for (int i = 0; i < 10; i++) {
  //   led_string += led_pixels_[i] > 0 ? "*" : " ";
  //   led_string += " ";
  // }

  // ROS_DEBUG_STREAM("LED strip status: " << led_string);
#endif

  return true;
}

::std::string LedStrip::GetStrip() {
  ::std::ostringstream strip_string;
  for (int i = 0; i < kNumberOfLeds; i++) {
    int red = led_pixels_[i] & 0xFF;
    int green = (led_pixels_[i] >> 8) & 0xFF;
    int blue = (led_pixels_[i] >> 16) & 0xFF;

    strip_string << ::std::endl
                 << ::std::setw(4) << red << ::std::setw(4) << green
                 << ::std::setw(4) << blue;
  }

  return strip_string.str();
}

void LedStrip::SetLed(int led, unsigned char r, unsigned char g,
                      unsigned char b) {

  ws2811_led_t led_color = (b << 16) | (g << 8) | r;
  led_pixels_[led] = led_color;
}

} // namespace led_strip
} // namespace gpio_writer
} // namespace io
} // namespace controls
} // namespace src
