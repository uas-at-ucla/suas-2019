#include "led_strip.h"

namespace src {
namespace controls {
namespace io {
namespace led_strip {

LedStrip::LedStrip() :
    next_led_write_(::lib::phased_loop::GetCurrentTime()),
    startup_sequence_frame_(0),
    battery_percentage_(0.0),
    armed_(false),
    alarm_(false),
    blank_(false) {

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
      0,             // Brightness value
      0,             // White shift value
      0,             // Red shift value
      0,             // Green shift value
      0,             // Blue shift value
      nullptr        // Gamma correction
  };

  leds_ = {
      0,                       // Render wait time
      nullptr,                 // Device
      nullptr,                 // Raspi hardware information
      WS2811_TARGET_FREQ,      // Required output frequency
      kLedStripDma,            // DMA number
      {channel_0_, channel_1_} // Channels
  };

#ifdef RASPI_DEPLOYMENT
  ws2811_return_t ret = ws2811_init(&leds_);

  if (ret != WS2811_SUCCESS) {
    ::std::cout << "ERROR!\n";
    return;
  }
#endif
}

LedStrip::~LedStrip() {
  delete led_pixels_;

#ifdef RASPI_DEPLOYMENT
  ws2811_fini(&leds_);
#endif
}

bool LedStrip::Render(bool force) {
  // 11 arm leds
  // 10 gps leds
  // 10 battery leds

  if (!force && ::lib::phased_loop::GetCurrentTime() < next_led_write_) {
    return true;
  }

  next_led_write_ = ::lib::phased_loop::GetCurrentTime() + kLedWriterPeriod;

  // Render the arming state indicator.
  {
    bool should_blink =
        !armed_ && ::std::fmod(::lib::phased_loop::GetCurrentTime(),
                               1.0 / kDisarmedBlinkFrequency) <
                       1.0 / (2 * kDisarmedBlinkFrequency);

    for (int i = 0; i < kNumberOfArmDisarmLeds; i++) {
      int current_led = i;
      if (armed_) {
        SetLed(current_led, 255, 0, 0);
      } else if (should_blink) {
        SetLed(current_led, 0, 255, 0);
      } else {
        SetLed(current_led, 0, 0, 0);
      }
    }
  }

  for (int i = kNumberOfArmDisarmLeds;
       i < kNumberOfLeds - kNumberOfBatteryLevelLeds + 1; i++) {
    SetLed(i, 10, 10, 10);
  }

  // Render the battery level indicator.
  {
    int solid =
        ::std::ceil((battery_percentage_ - 0.1) * kNumberOfBatteryLevelLeds);
    bool blinky = ::std::fmod(battery_percentage_, 0.2) < 0.1;

    for (int i = 0; i < kNumberOfBatteryLevelLeds; i++) {
      int current_led = kNumberOfLeds - i;

      bool should_blink = i == solid && blinky &&
                          ::std::fmod(::lib::phased_loop::GetCurrentTime(),
                                      1.0 / kBatteryBlinkFrequency) <
                              1.0 / (2 * kBatteryBlinkFrequency);
      if (i < solid || should_blink) {
        // LED should be solid.
        SetLed(current_led, 0, 0, 255);
      } else {
        // LED should be off.
        SetLed(current_led, 0, 0, 0);
      }
    }
  }

  // Override everything and rapidly flash if there hasn't been an IMU message
  // from the Pixhawk recently.
  if (last_imu_ + kImuTimeout < ::lib::phased_loop::GetCurrentTime()) {
    bool should_light =
        ::std::fmod(::lib::phased_loop::GetCurrentTime(),
                    1.0 / kFlightControllerDisconnectBlinkFrequency) <
        1.0 / (2 * kFlightControllerDisconnectBlinkFrequency);

    for (int i = 0; i < kNumberOfLeds; i++) {
      if (should_light) {
        SetLed(i, 255, 0, 0);
      } else {
        SetLed(i, 255, 255, 255);
      }

      // If an alarm is active, override the current LED value.
      if (alarm_) {
        SetLed(i, 255, 255, 255);
      }
    }
  }

  // Run startup sequence.
  if (startup_sequence_frame_ <
      kLedWriterFramesPerSecond * kStartupSequenceSeconds) {
    for (int i = 0; i < kNumberOfLeds; i++) {
      bool should_light =
          i <= kNumberOfLeds * startup_sequence_frame_ /
                   (kLedWriterFramesPerSecond * kStartupSequenceSeconds);

      if (should_light) {
        SetLed(i, 255, 255, 255);
      } else {
        SetLed(i, 0, 0, 0);
      }
    }
    startup_sequence_frame_++;
  }

  if (blank_) {
    for (int i = 0; i < kNumberOfLeds; i++) {
      SetLed(i, 0, 0, 0);
    }
  }

#ifdef RASPI_DEPLOYMENT
  ws2811_return_t ret = ws2811_render(&leds_);

  if (ret != WS2811_SUCCESS) {
    ::std::cout << "Could not render\n";
    return false;
  }
#endif

  return true;
}

::std::string LedStrip::GetStrip() {
  ::std::ostringstream strip_string;
  for (int i = 0; i < kNumberOfLeds; i++) {
    int red = leds_.channel[0].leds[i] & 0xFF;
    int green = (leds_.channel[0].leds[i] >> 8) & 0xFF;
    int blue = (leds_.channel[0].leds[i] >> 16) & 0xFF;

    strip_string << ::std::endl
                 << ::std::setw(4) << red << ::std::setw(4) << green
                 << ::std::setw(4) << blue;
  }

  return strip_string.str();
}

void LedStrip::SetLed(int led, unsigned char r, unsigned char g,
                      unsigned char b) {

  ws2811_led_t led_color = (b << 16) | (g << 8) | r;
  leds_.channel[0].leds[led] = led_color;
}

} // namespace led_strip
} // namespace io
} // namespace controls
} // namespace src
