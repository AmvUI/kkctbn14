/*
 * The Arduino program used in the YouTube video http://www.youtube.com/watch?v=w6tEguPEOkg
 *
 * This program allow to move a ball to a target position. This is
 * done by using the ball position given by OpenCV and updating the
 * rails altitude. The communication is done using a serial connection
 * (through USB or native).
 *
 * See the documentation to get the OpenCV program.
 *
 * Author: Frédéric Jolliton <frederic@jolliton.com>
 * Date: april 1st, 2011
 * Documentation: http://doc.tuxee.net/ball
 */

#include "WProgram.h"
#include "Servo/Servo.h"

Servo servo;

void setup()
{
  Serial.begin(9600);
  servo.attach(11);
}

// The position of the servo motor for which the rails are horizontal.
static const int center = 1150;

// The target position (the webcam captures 640 pixels wide image, so
// 320 is the middle.)
static const int target = 320;

// Duration between update (in milliseconds)
static const float dt = 50.;

static float current = 320.;
static float last_error = 0;
static float filt_error = 0.0;

static int value = 0;

// integral term (here because it is accumulating)
static float i = 0.0;

//-------- PID parameters --------

// See http://en.wikipedia.org/wiki/PID_controller

// These values must be chosen CAREFULLY. The strategy to find good
// values is to set `ci' and `cd' to 0.0, then try to find a value of
// `cp' that works the best (without too much oscillation) then, from
// that, lower `cp' and increase `cd' until the system is able to
// stalibilize more quickly. Increase `ci' if the system take time to
// move to the target position. There are more complexes method to
// find the "right" values.

static const float pval = 0.12;
static const float ival = 0.00018;
static const float dval = 1350.;

void loop()
{
  //-------- Receive position from the PC (OpenCV) --------

  // Note that we don't necessary read a whole line at once. The value
  // is accumulated until it is read entirely.
  while (Serial.available()) {
    int c = Serial.read();
    if (c >= '0' && c <= '9') {
      value = 10 * value + (c - '0');
    } else if (c == '\n') {
      current = current + (value - current) * .3;
      value = 0;
    } else if (c == '\r') {
      // skip
    } else {
      value = 0;
    }
  }

  // the error we want to correct
  int                 error = target - current;

  // low-pass filter to eliminate fast variation of the input
  // position.
  filt_error = filt_error + (error - filt_error) * 0.7;

  // the proportional term
  float               p = pval * error;

  // update the integral term
  i += ival * dt * error;

  // the derivative term
  float               d = dval * (filt_error - last_error) / dt;
  last_error = filt_error;

  // the PID value
  int                 delta = p + i + d;

  // clamp delta
  if (delta < -200) {
    delta = -200;
  } else if (delta > 400) {
    delta = 400;
  }

  // Update the servo position
  servo.writeMicroseconds(center + delta);

  // Wait a bit before iterating again.
  delay(dt);
}
