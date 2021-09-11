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
byte keys[KMS*KMS] = {0, };

/*
00 C 1
01
02 C 2
03 C 3
04 C 4
05

10 P 2
11 P 1
12 P 3
13 P 4
14 P 5
15

20 R 5
21 R L
22 R 6
23 R 7
24 R 8
25 R U

30 R 1
31 R D
32 R 2
33 R 3
34 R 4
35 R R

40 L 1
41 L R
42 L 2
43 L 3
44 L 4
45 L D

50 L 5
51 L U
52 L 6
53 L 7
54 L 8
55 L L

*/

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
      keys[col*KMS+row]=!digitalRead(rows[row]);
    }
    pinMode(cols[col], INPUT_PULLUP);
  }

  for (byte col=0; col<KMS; col++) {
    for (byte row=0; row<KMS; row++) {
      Serial.print(keys[col*KMS+row]);
    }
    Serial.print(" ");
  }

/*
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
*/
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
