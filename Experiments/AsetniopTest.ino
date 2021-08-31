#include "Asetniop.h"

void setup() {
  Serial.begin(9600);
  for(int pin=2; pin<12; pin++)
    pinMode(pin, INPUT_PULLUP);
}

unsigned char cumulativeMask = 0x00;

void loop() {
  unsigned char pressedMask = 0x00;
  pressedMask |= (!digitalRead(2)) << 7;
  pressedMask |= (!digitalRead(3)) << 6;
  pressedMask |= (!digitalRead(4)) << 5;
  pressedMask |= (!digitalRead(5)) << 4;
  pressedMask |= (!digitalRead(8)) << 3;
  pressedMask |= (!digitalRead(9)) << 2;
  pressedMask |= (!digitalRead(10)) << 1;
  pressedMask |= (!digitalRead(11)) << 0;

  if (pressedMask == 0 && cumulativeMask != 0) {
    Serial.println(asetniop_lut[cumulativeMask]);
    delay(50);
    cumulativeMask = 0;
  } else {
    cumulativeMask |= pressedMask;
  }
}
