// Implements two Joysticks, one Keyboard and a Mouse running
// on an Arduino Due connected to my arcade machine
// 
// by Renato Grottesi
// 09/08/2021
//--------------------------------------------------------------------

#include <Keyboard.h>
#include <Mouse.h>
/* From: https://github.com/MHeironimus/ArduinoJoystickLibrary */
#include <Joystick.h>
#include "Asetniop.h"

Joystick_ JoystickLeft(0x03);
Joystick_ JoystickRight(0x04);

// Keyboard Matrix Size
#define KMS 6
const byte rows[KMS] = {42, 44, 46, 48, 50, 52};
const byte cols[KMS] = {43, 45, 47, 49, 51, 53};

void setup() {
  Serial.begin(9600);

  // Initialize the joysticks
  JoystickLeft.begin();
  JoystickRight.begin();

  // Set all pins to High Impedance
  for (byte row=0; row<KMS; row++) {
    pinMode(rows[row], INPUT_PULLUP);
  }
  for (byte col=0; col<KMS; col++) {
    pinMode(cols[col], INPUT_PULLUP);
  }
}

void loop() {
  for (byte col=0; col<KMS; col++) {
    pinMode(cols[col], OUTPUT);
    digitalWrite(cols[col], LOW);
    for (byte row=0; row<KMS; row++) {
      Serial.print(digitalRead(rows[row]));
    }
    Serial.print(" ");
    pinMode(cols[col], INPUT_PULLUP);
  }

  Serial.print("    ");

  for (byte row=0; row<KMS; row++) {
    pinMode(rows[row], OUTPUT);
    digitalWrite(rows[row], LOW);
    for (byte col=0; col<KMS; col++) {
      Serial.print(digitalRead(cols[col]));
    }
    Serial.print(" ");
    pinMode(rows[row], INPUT_PULLUP);
  }

  Serial.println(" ");

/*  for (int index = 0; index < 4; index++)
  {
    int currentButtonState = 0;
    if (currentButtonState != lastButtonState[index])
    {
      if (index < 2) {
        JoystickLeft.setButton(index, currentButtonState);
        JoystickRight.setButton(index+4, currentButtonState);
        lastButtonState[index] = currentButtonState;
      } else {
        if (currentButtonState) {
          Keyboard.write(47 + index);
          Mouse.move(5, 5, 0);
          delay(500);
        }
      }
    }
  }
*/
}
