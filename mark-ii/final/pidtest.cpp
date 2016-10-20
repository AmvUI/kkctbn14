 #include <sys/time.h> 
  #include <vector>
#include <opencv/cv.h>
#include <opencv/cvaux.h>
#include <opencv/highgui.h>
#include <iostream>
#include <list>

#include <opencv2/video/background_segm.hpp>
#include <opencv2/legacy/blobtrack.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc_c.h>

#include <opencv2/core/core.hpp>
#include <opencv2/video/tracking.hpp>

#include <math.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <termios.h>
 

//-------- PID parameters --------

// See http://en.wikipedia.org/wiki/PID_controller

// These values must be chosen CAREFULLY. The strategy to find good
// values is to set `ci' and `cd' to 0.0, then try to find a value of
// `cp' that works the best (without too much oscillation) then, from
// that, lower `cp' and increase `cd' until the system is able to
// stalibilize more quickly. Increase `ci' if the system take time to
// move to the target position. There are more complexes method to
// find the "right" values.

static double cp = 0.2;
static double ci = 0.0;
static double cd = 0.02;

/*
 * Get the current time in milliseconds
 */

static double getTime()
{
  struct timeval tim;
  gettimeofday(&tim, 0);
  return tim.tv_sec+(tim.tv_usec/1000000.0);
}
  
 int main () { 	


  double              angle = 1500.0; // the position of the camera
  int                 tcolor = 0; // target color - Used to switch to predefined hue levels.
  double              last_time = 0.0; // last time we updated PID
  int                 last_known_x = 320; // last known position of the target.
  double              last_error = 0.0;
  double              i = 0.0; // integral term (here because it is accumulating)
  int                 last_sent_value = -1;
  int                 target = 320; // posisi target
  

   

    //-------- PID processing --------

    // If the object is out of the window, use last known position to
    // find it.
    if (x < 0 || x > 640) {
      x = 2.5 * (last_known_x - 320) + 320;
    } else {
      last_known_x = x;
    }

    double              time = getTime();
    double              dt = time - last_time;
    if (last_time == 0.0){dt = 1.0;}
    last_time = time;

    double              error = x - target;
    double              p = error * cp;
    i += error * dt * ci;

    // Clamp integral term
    if (i > 30.0){i = 30.0;}
    else if (i < -30.0){i = -30.0;}

    // the derivative terms
    double              d = (error - last_error) / dt * cd;
    last_error = error;

    // the PID value
    double              pid = p + i + d;

    // Clamp PID
    if (pid < -100){pid = -100;}
    else if (pid > 100){pid = 100;}

    // Update the position from the PID value.
    angle += pid;

    // Clamp angle
    if (angle < 0)  
      angle = 0;
    else if (angle > 2000)
      angle = 2000;

 return 0;
}
