/**
 * Implements a USB HID mouse that reads the X/Y and buttons
 * values from a PS/2 trackball connected to an Arduino Due
 * using the PS2Mouse library available from
 *   http://github.com/kristopher/PS2-Mouse-Arduino/
 */

#include <Mouse.h>
#include <PS2Mouse.h>
#define TRACKBALL_DATA 5
#define TRACKBALL_CLOCK 6
PS2Mouse trackball(TRACKBALL_CLOCK, TRACKBALL_DATA, STREAM);

void setup()
{
  trackball.initialize();
  Mouse.begin();
}

void loop()
{
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
}
