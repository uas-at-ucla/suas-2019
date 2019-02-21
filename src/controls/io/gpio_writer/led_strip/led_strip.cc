#include "led_strip.h"

namespace src {
namespace controls {
namespace io {
namespace gpio_writer {
namespace led_strip {

LedStrip::LedStrip() : battery_percentage_(0.0) {
  ws2811_led_t *leds =
      static_cast<ws2811_led_t *>(malloc(sizeof(ws2811_led_t) * kNumberOfLeds));

  channel_0_ = {
      kLedStripGpioPin, // GPIO number
      0,                // Invert output signal
      kNumberOfLeds,    // Number of LEDs, 0 if channel is unused.
      kLedStripType,    // Strip color layout
      leds,             // LED buffer
      255,              // Brightness value
      0,                // White shift value
      0,                // Red shift value
      0,                // Green shift value
      0,                // Blue shift value
      nullptr           // Gamma correction
  };

  channel_1_ = {
      kLedStripGpioPin, // GPIO number
      0,                // Invert output signal
      0,                // Number of LEDs, 0 if channel is unused.
      kLedStripType,    // Strip color layout
      leds,             // LED buffer
      255,              // Brightness value
      0,                // White shift value
      0,                // Red shift value
      0,                // Green shift value
      0,                // Blue shift value
      nullptr           // Gamma correction
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
#ifdef UAS_AT_UCLA_DEPLOYMENT
  ws2811_fini(&leds_);
#endif
}

bool LedStrip::Render() {
  int solid = ::std::ceil((battery_percentage_ - 0.1) / 0.2);
  bool blinky = ::std::fmod(battery_percentage_, 0.2) < 0.1;

#ifndef UAS_AT_UCLA_DEPLOYMENT
  bool set[5];
#endif

  for (int i = 0; i < 5; i++) {
    bool should_blink = i == solid && blinky &&
                        ::std::fmod(::lib::phased_loop::GetCurrentTime(),
                                    1.0 / kBatteryBlinkFrequency) <
                            1.0 / (2 * kBatteryBlinkFrequency);
    if (i < solid || should_blink) {
      // LED should be solid.
      SetLed(i, 0, 255, 0);
      SetLed(9 - i, 0, 255, 0);
#ifndef UAS_AT_UCLA_DEPLOYMENT
      set[i] = true;
#endif
    } else {
      // LED should be off.
      SetLed(i, 0, 0, 0);
      SetLed(9 - i, 0, 0, 0);
#ifndef UAS_AT_UCLA_DEPLOYMENT
      set[i] = false;
#endif
    }
  }

#ifdef UAS_AT_UCLA_DEPLOYMENT
  ws2811_return_t ret = ws2811_render(&leds_);

  if (ret != WS2811_SUCCESS) {
    return false;
  }
#else
  for (int i = 0; i < 5; i++) {
    ::std::cout << set[i] << " ";
  }
  ::std::cout << ::std::endl;
#endif

  return true;
}

void LedStrip::SetLed(int led, unsigned char r, unsigned char g,
                      unsigned char b) {

#ifdef UAS_AT_UCLA_DEPLOYMENT
  ws2811_led_t led_color = (b << 16) | (g << 8) | r;

  ::std::cout << "setting led " << led << ::std::endl;
  leds_.channel[0].leds[led] = led_color;

#else
  (void)led;
  (void)r;
  (void)g;
  (void)b;
#endif
}

} // namespace led_strip
} // namespace gpio_writer
} // namespace io
} // namespace controls
} // namespace src
