// Implements two Joysticks, one Keyboard and a Mouse running
// on an Arduino Due connected to my arcade machine
// 
// by Renato Grottesi
// 09/08/2021
//--------------------------------------------------------------------

#include <Keyboard.h>
#include <Joystick.h>

Joystick_ JoystickLeft(0x03);
Joystick_ JoystickRight(0x04);

void setup() {
  // Initialize Button Pins
  pinMode(3, INPUT_PULLUP);
  pinMode(4, INPUT_PULLUP);
  pinMode(5, INPUT_PULLUP);
  pinMode(6, INPUT_PULLUP);

  // Initialize the joysticks
  JoystickLeft.begin();
  JoystickRight.begin();
}

// Constant that maps the phyical pin to the joystick button.
const int pinToButtonMap = 3;

// Last state of the button
int lastButtonState[4] = {0,0,0,0};

void loop() {

  // Read pin values
  for (int index = 0; index < 4; index++)
  {
    int currentButtonState = !digitalRead(index + pinToButtonMap);
    if (currentButtonState != lastButtonState[index])
    {
      if (index < 2) {
        JoystickLeft.setButton(index, currentButtonState);
        JoystickRight.setButton(index+4, currentButtonState);
        lastButtonState[index] = currentButtonState;
      } else {
        if (currentButtonState) {
          Keyboard.write(47 + index);
          delay(500);
        }
      }
    }
  }

  delay(100);
}
