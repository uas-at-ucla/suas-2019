#include "led_strip.h"

namespace src {
namespace controls {
namespace io {
namespace gpio_writer {
namespace led_strip {

LedStrip::LedStrip() {
  ws2811_led_t leds[kNumberOfLeds];

  ws2811_channel_t channel_0 = {
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

  ws2811_channel_t channel_1 = {
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
      {channel_0, channel_1}    // Channels
  };

  ws2811_return_t ret = ws2811_init(&leds_);

  if (ret != WS2811_SUCCESS) {
    return;
  }
}

LedStrip::~LedStrip() { ws2811_fini(&leds_); }

bool LedStrip::Render() {
  ws2811_return_t ret = ws2811_render(&leds_);

  if (ret != WS2811_SUCCESS) {
    return false;
  }

  return true;
}

void LedStrip::SetLed(int led, unsigned char r, unsigned char g,
                      unsigned char b) {
  ws2811_led_t led_color = (b << 16) | (g << 8) | r;

  leds_.channel[0].leds[led] = led_color;
}

} // namespace led_strip
} // namespace gpio_writer
} // namespace io
} // namespace controls
} // namespace src
