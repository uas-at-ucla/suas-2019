#include <stdio.h>
#include <stdlib.h>
#include <wiringPi.h>

// LED Pin - wiringPi pin 0 is BCM_GPIO 17.

#define LED 0

int main(void) {
  printf("Raspberry Pi blink\n");

  wiringPiSetup();
  pinMode(LED, OUTPUT);
  piHiPri(1);

  for (;;) {
    digitalWrite(LED, HIGH); // On
    delayMicroseconds(1e6 / 100);
    digitalWrite(LED, LOW); // Off
    delayMicroseconds(1e6 / 100);
  }
  return 0;
}
