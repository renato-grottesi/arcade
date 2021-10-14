// Implements two Joysticks, one Keyboard and a Mouse running
// on an Arduino Due connected to my arcade machine
// 
// by Renato Grottesi
// 09/08/2021
//--------------------------------------------------------------------

#include <Keyboard.h>
#include <Mouse.h>
/* From: http://github.com/kristopher/PS2-Mouse-Arduino */
#include <PS2Mouse.h>
/* From: https://github.com/MHeironimus/ArduinoJoystickLibrary */
#include <Joystick.h>
/* From: https://github.com/ElectronicCats/mpu6050 */
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
/* Local ASETNIOP implementation */
#include "Asetniop.h"

MPU6050 accelgyro;

Joystick_ JoystickLeft(0x03);
Joystick_ JoystickRight(0x04);

// Keyboard Matrix Size
#define KMS 6
const byte rows[KMS] = {42, 44, 46, 48, 50, 52};
const byte cols[KMS] = {43, 45, 47, 49, 51, 53};
byte keys[KMS*KMS] = {0, };

/* Key Names */
enum struct KN: byte{
  C1, N1, C2, C3, C4, N2,
  P2, P1, P3, P4, P5, N3,
  R8, RL, R6, R5, R7, RU,
  R4, RD, R2, R1, R3, RR,
  L4, LR, L2, L3, L1, LD,
  L8, LU, L6, L7, L5, LL
};

enum struct Modes: byte{
  PLAYER_ONE,
  PLAYER_TWO,
  ASETNIOP,
  KEYBOARD,
  PINBALL_JOY,
  PINBALL_PC,
};

#define KEY(kn) keys[static_cast<byte>(KN::kn)]

#define TRACKBALL_DATA 5
#define TRACKBALL_CLOCK 6
PS2Mouse trackball(TRACKBALL_CLOCK, TRACKBALL_DATA, STREAM);

unsigned char cumulativeMask = 0x00;

Modes mode = Modes::PLAYER_TWO;

void setup() {
  Serial.begin(9600);

  // The accelerometer needs I2C
  Wire.begin();
  // Initialize and verify the accelerometer
  accelgyro.initialize();
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  // Initialize the joysticks
  JoystickLeft.begin();
  JoystickRight.begin();

  // Set all rows as input pullup, connected directly to the buttons
  for (byte row=0; row<KMS; row++) {
    pinMode(rows[row], INPUT_PULLUP);
  }
  // Set all columns to high impedance, connected to the buttons via diodes
  for (byte col=0; col<KMS; col++) {
    pinMode(cols[col], INPUT);
  }

  trackball.initialize();
  Mouse.begin();
}

void loop() {
  int16_t ax, ay, az;
  accelgyro.getAcceleration(&ax, &ay, &az);
  // TODO Filter and send as pinball mode events
//  Serial.println(ax); // right/left nudge
//  Serial.println(az); // front push/pull

  for (byte col=0; col<KMS; col++) {
    pinMode(cols[col], OUTPUT);
    digitalWrite(cols[col], LOW);
    for (byte row=0; row<KMS; row++) {
      keys[col*KMS+row]=!digitalRead(rows[row]);
    }
    pinMode(cols[col], INPUT);
  }

/*
  for (byte col=0; col<KMS; col++) {
    for (byte row=0; row<KMS; row++) {
      Serial.print(keys[col*KMS+row]);
    }
    Serial.print(" ");
  }
  Serial.println(" ");
*/

  if(KEY(P2) && KEY(C1)) mode = Modes::PLAYER_ONE;
  if(KEY(P3) && KEY(C1)) mode = Modes::ASETNIOP;
  if(KEY(P2) && KEY(P3) && KEY(C1)) mode = Modes::PLAYER_TWO;
  if(KEY(P2) && KEY(P3) && KEY(P4) && KEY(P5)) mode = Modes::PINBALL_JOY;

  //JoystickRight.setXAxis(512);
  //JoystickRight.setYAxis(512);
  JoystickRight.setZAxis(512);
  JoystickRight.setRxAxis(512);
  JoystickRight.setRyAxis(512);
  JoystickRight.setRzAxis(512);

  //JoystickLeft.setXAxis(512);
  //JoystickLeft.setYAxis(512);
  JoystickLeft.setZAxis(512);
  JoystickLeft.setRxAxis(512);
  JoystickLeft.setRyAxis(512);
  JoystickLeft.setRzAxis(512);

  if(mode == Modes::PINBALL_JOY) {
    JoystickLeft.setXAxis(512+(KEY(LR)-KEY(LL))*512);
    JoystickLeft.setYAxis(512+(KEY(LD)-KEY(LU))*512);
    JoystickLeft.setButton(0, KEY(P2));
    JoystickLeft.setButton(1, KEY(P3));
    JoystickLeft.setButton(2, KEY(P4));
    JoystickLeft.setButton(3, KEY(L1));
    JoystickLeft.setButton(4, KEY(L7));
    JoystickLeft.setButton(5, KEY(L8));
    JoystickLeft.setButton(6, KEY(P1));
    JoystickLeft.setButton(7, KEY(P5));
    JoystickLeft.setButton(8, KEY(C1));
    JoystickLeft.setButton(9, KEY(C2));
  }

  if(mode == Modes::PLAYER_ONE) {
    JoystickLeft.setXAxis(512+(KEY(LR)-KEY(LL))*512);
    JoystickLeft.setYAxis(512+(KEY(LD)-KEY(LU))*512);
    JoystickLeft.setButton(0, KEY(L6));
    JoystickLeft.setButton(1, KEY(L2));
    JoystickLeft.setButton(2, KEY(L5));
    JoystickLeft.setButton(3, KEY(L1));
    JoystickLeft.setButton(4, KEY(L7));
    JoystickLeft.setButton(5, KEY(L8));
    JoystickLeft.setButton(6, KEY(L3));
    JoystickLeft.setButton(7, KEY(L4));
    JoystickLeft.setButton(8, KEY(C1));
    JoystickLeft.setButton(9, KEY(C2));
  }

  if(mode == Modes::PLAYER_TWO) {
    JoystickLeft.setXAxis(512+(KEY(LR)-KEY(LL))*512);
    JoystickLeft.setYAxis(512+(KEY(LD)-KEY(LU))*512);
    JoystickLeft.setButton(0, KEY(L6));
    JoystickLeft.setButton(1, KEY(L2));
    JoystickLeft.setButton(2, KEY(L5));
    JoystickLeft.setButton(3, KEY(L1));
    JoystickLeft.setButton(4, KEY(L7));
    JoystickLeft.setButton(5, KEY(L8));
    JoystickLeft.setButton(6, KEY(L3));
    JoystickLeft.setButton(7, KEY(L4));
    JoystickLeft.setButton(8, KEY(C1));
    JoystickLeft.setButton(9, KEY(C2));
  
    JoystickRight.setXAxis(512+(KEY(RR)-KEY(RL))*512);
    JoystickRight.setYAxis(512+(KEY(RD)-KEY(RU))*512);
    JoystickRight.setButton(0, KEY(R6));
    JoystickRight.setButton(1, KEY(R2));
    JoystickRight.setButton(2, KEY(R5));
    JoystickRight.setButton(3, KEY(R1));
    JoystickRight.setButton(4, KEY(R7));
    JoystickRight.setButton(5, KEY(R8));
    JoystickRight.setButton(6, KEY(R3));
    JoystickRight.setButton(7, KEY(R4));
    JoystickRight.setButton(8, KEY(C3));
    JoystickRight.setButton(9, KEY(C4));
  }

  /**
   * data[0]): Status Byte: 0x01=left button, 0x10=right button
   * data[1]): Movement Data: + is down - is up
   * data[2]): Movement Data: - is left + is right
  */
  int16_t data[3];
  trackball.report(data);

  Mouse.move(data[2], data[1], 0);
  if((data[0]&0x1) && !(Mouse.isPressed(MOUSE_LEFT)))    Mouse.press(MOUSE_LEFT);
  if(!(data[0]&0x1) && (Mouse.isPressed(MOUSE_LEFT)))    Mouse.release(MOUSE_LEFT);
  data[0]>>=1;
  if((data[0]&0x1) && !(Mouse.isPressed(MOUSE_RIGHT)))   Mouse.press(MOUSE_RIGHT);
  if(!(data[0]&0x1) && (Mouse.isPressed(MOUSE_RIGHT)))   Mouse.release(MOUSE_RIGHT);

  if(mode == Modes::ASETNIOP) {
    unsigned char pressedMask = 0x00;
    pressedMask |= (KEY(L1)) << 7;
    pressedMask |= (KEY(L2)) << 6;
    pressedMask |= (KEY(L3)) << 5;
    pressedMask |= (KEY(L4)) << 4;
    pressedMask |= (KEY(R1)) << 3;
    pressedMask |= (KEY(R2)) << 2;
    pressedMask |= (KEY(R3)) << 1;
    pressedMask |= (KEY(R4)) << 0;
  
    if (pressedMask == 0 && cumulativeMask != 0) {
      Keyboard.write(asetniop_lut[cumulativeMask]);
      delay(50);
      cumulativeMask = 0;
    } else {
      cumulativeMask |= pressedMask;
    }
  }

}
