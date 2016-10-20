/*
 * The program used in the YouTube video http://www.youtube.com/watch?v=w6tEguPEOkg
 *
 * This program allow to move a ball to a target position. This is
 * done by using the ball position given by OpenCV and updating the
 * rails altitude. The communication is done using a serial connection
 * (through USB or native).
 *
 * To compile it under Linux (assuming OpenCV was previously installed):
 *
 *   g++ -O2 -W -Wall -lhighgui ball.cc -o ball
 *
 * See the documentation to get the Arduino program.
 *
 * Author: Frédéric Jolliton <frederic@jolliton.com>
 * Date: april 1st, 2011
 * Documentation: http://doc.tuxee.net/ball
 */
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

int main(int argc, char* argv[])
{
  CvCapture*          capture = cvCreateCameraCapture(0);
  if (capture == 0) {
    std::cerr << "Failed to open the camera.\n";
    exit(1);
  }

  /*
   * Window to display the input image.
   */
  cvNamedWindow("RGB", CV_WINDOW_AUTOSIZE);
  cvMoveWindow("RGB", 0, 0);

  // The hue range to select.
  int                 hue_start_level = 108;
  int                 hue_stop_level = 120;
  int                 hue_overlay = 0;

  int                 sat_start_level = 180;
  int                 sat_stop_level = 256;
  int                 sat_overlay = 0;

  int                 lum_start_level = 0;
  int                 lum_stop_level = 256;
  int                 lum_overlay = 0;

  int                 mask_overlay = 1;

  int                 erode_level = 3;

  // Read previous parameters
  FILE*               f = fopen("ball-param", "r");
  if (f != 0) {
    fscanf(f, "%d %d %d %d %d %d %d\n",
           &hue_start_level, &hue_stop_level,
           &sat_start_level, &sat_stop_level,
           &lum_start_level, &lum_stop_level,
           &erode_level);
    fclose(f);
    printf("Settings loaded.\n");
  }

  /*
   * Window to display the mask (the selected part of the input image)
   */
  cvNamedWindow("Mask", CV_WINDOW_AUTOSIZE);
  cvMoveWindow("Mask", 0, 505);

  /*
   * The settings window
   */
  cvNamedWindow("Settings", 0);
  cvMoveWindow("Settings", 646, 505);

  // Color selection
  cvCreateTrackbar("Hue level (start)", "Settings", &hue_start_level, 256, 0);
  cvCreateTrackbar("Hue level (stop)", "Settings", &hue_stop_level, 256, 0);

  // Saturation selection
  cvCreateTrackbar("Saturation level (start)", "Settings", &sat_start_level, 256, 0);
  cvCreateTrackbar("Saturation level (stop)", "Settings", &sat_stop_level, 256, 0);

  // Luminosity selection
  cvCreateTrackbar("Luminosity level (start)", "Settings", &lum_start_level, 256, 0);
  cvCreateTrackbar("Luminosity level (stop)", "Settings", &lum_stop_level, 256, 0);

  // Switch to display the selection on the various HSV channels
  cvCreateTrackbar("Hue overlay", "Settings", &hue_overlay, 1, 0);
  cvCreateTrackbar("Saturation overlay", "Settings", &sat_overlay, 1, 0);
  cvCreateTrackbar("Luminosity overlay", "Settings", &lum_overlay, 1, 0);
  cvCreateTrackbar("Mask overlay", "Settings", &mask_overlay, 1, 0);

  cvCreateTrackbar("Erode level", "Settings", &erode_level, 4, 0);

  cvResizeWindow("Settings", 850, 400);

  /*
   * Window to display the tracking state
   */
  cvNamedWindow("Track", CV_WINDOW_AUTOSIZE);
  cvMoveWindow("Track", 646, 0);

  IplImage*           chan_h = 0;
  IplImage*           chan_s = 0;
  IplImage*           chan_v = 0;
  IplImage*           chan3 = 0;
  IplImage*           chan4 = 0;
  IplImage*           chan5 = 0;
  IplImage*           mask = 0;
  IplImage*           hsv = 0;
  IplImage*           monitor = 0;
  IplImage*           result = 0;
  int                 last_value = -1;
  unsigned long       iteration = 0;

  FILE*               serial = fopen("/dev/ttyACM0", "w");
  if (serial == 0) {
    printf("Failed to open serial port\n");
  }
  sleep(1);

  for (;;) {

    //-------- Get the input image --------

    IplImage*           frame = cvQueryFrame(capture);
    if (frame == 0) break;
    cvShowImage("RGB", frame);

    //-------- Allocate images --------

    if (hsv == 0) {
      // Allocate images if it is the first iteration.
      hsv     = cvCreateImage(cvGetSize(frame), 8, 3); // 8 bits, 3 channels
      chan_h  = cvCreateImage(cvGetSize(frame), 8, 1); // 8 bits, 1 channels
      chan_s  = cvCreateImage(cvGetSize(frame), 8, 1);
      chan_v  = cvCreateImage(cvGetSize(frame), 8, 1);
      chan3   = cvCreateImage(cvGetSize(frame), 8, 1);
      chan4   = cvCreateImage(cvGetSize(frame), 8, 1);
      chan5   = cvCreateImage(cvGetSize(frame), 8, 1);
      mask    = cvCreateImage(cvGetSize(frame), 8, 1);
      monitor = cvCreateImage(cvGetSize(frame), 8, 3);
      result  = cvCreateImage(cvGetSize(frame), 8, 3);
    }

    //-------- HSV extraction --------

    cvCvtColor(frame, hsv, CV_BGR2HSV); // convert to HSV (Hue-Saturation-Value)
    cvSplit(hsv, chan_h, chan_s, chan_v, 0); // extract Hue & Saturation

    //-------- Create mask to isolate objects --------

    if (hue_start_level <= hue_stop_level) {
      cvInRangeS(chan_h, cvScalar(hue_start_level), cvScalar(hue_stop_level), chan3);
    } else {
      cvInRangeS(chan_h, cvScalar(hue_stop_level), cvScalar(hue_start_level), chan3);
      cvSubRS(chan3, cvScalar(255), chan3);
    }
    cvInRangeS(chan_s, cvScalar(sat_start_level), cvScalar(sat_stop_level), chan4);
    cvInRangeS(chan_v, cvScalar(lum_start_level), cvScalar(lum_stop_level), chan5);
    cvAnd(chan3, chan4, mask); // Merge masks
    cvAnd(mask, chan5, mask); // Merge masks
    cvErode(mask, mask, 0, erode_level + 1); // Suppress noise

    //-------- Find contours --------

    CvMemStorage*       storage = cvCreateMemStorage(0);
    CvSeq*              contour = 0;
    cvFindContours(mask, storage, &contour, sizeof(CvContour),
                   CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

    //-------- Update monitor frame --------

    cvConvertScale(frame, monitor, .3, 0); // faded out input
    for (int j = 0; j < 3; ++j) {
      if (lum_overlay && (iteration + j) % 3 == 0 )
        cvSet(monitor, cvScalar(0, 0, 255), chan5);
      if (sat_overlay && (iteration + j) % 3 == 1)
        cvSet(monitor, cvScalar(0, 255, 0), chan4);
      if (hue_overlay && (iteration + j) % 3 == 2)
        cvSet(monitor, cvScalar(255, 0, 0), chan3);
    }
    if (mask_overlay)
      cvSet(monitor, cvScalar(255, 255, 255), mask); // white overlay
    cvShowImage("Mask", monitor);

    //-------- Create output image --------

    cvCopy(frame, result, 0); // input image

    //-------- Draw contours --------

    CvPoint             first = cvPoint(-1, -1);
    CvPoint             last = cvPoint(-1, -1);
    CvScalar            line_color = cvScalar(0, 255, 255);

    int pos = -1;
    while (contour != 0) {
      CvRect              bb = cvBoundingRect(contour, 0);

      // Note: We ignore small contours.
      if (bb.width > 20 || bb.height > 20) {
        int                 x = bb.x + bb.width / 2;
        int                 y = bb.y + bb.height / 2;
        CvPoint             current = cvPoint(x, y);

        // Draw a circle around the selected object.
        cvCircle(result, cvPoint(x, y), 20, cvScalar(0, 0, 0), 3, CV_AA, 0);
        static int const    step = 8 * 4;
        for (int j = 0; j < step; j += 4) {
          double r = 20;
          double shift = iteration / 8.;
          int tx1 = 4 * (cos(j * 2. * M_PI / step + shift) * r + x);
          int ty1 = 4 * (sin(j * 2. * M_PI / step + shift) * r + y);
          int tx2 = 4 * (cos((j + 1) * 2. * M_PI / step + shift) * r + x);
          int ty2 = 4 * (sin((j + 1) * 2. * M_PI / step + shift) * r + y);
          cvLine(result, cvPoint(tx1, ty1), cvPoint(tx2, ty2), cvScalar(0, 255, 255), 2, CV_AA, 2);
        }
        pos = x;
        break;
      }
      contour = contour->h_next;
    }

    cvReleaseMemStorage(&storage);

    cvShowImage("Track", result);

    // Send the position to Arduino
    if (pos != -1 && serial != 0) {
      fprintf(serial, "%d\n", pos);
      printf("%d\n", pos);
    }

    //-------- Handle keyboard events --------

    int key = cvWaitKey(25);
    if (key == 27) break;
    switch (key) {
    case 'r':
      // Reset parameters
      cvSetTrackbarPos("Hue level (start)", "Settings", 0);
      cvSetTrackbarPos("Hue level (stop)", "Settings", 0);
      cvSetTrackbarPos("Saturation level (start)", "Settings", 256);
      cvSetTrackbarPos("Saturation level (stop)", "Settings", 256);
      cvSetTrackbarPos("Luminosity level (start)", "Settings", 256);
      cvSetTrackbarPos("Luminosity level (stop)", "Settings", 256);
      break;
    case 's':
      // Save current parameters
      {
        FILE*               f = fopen("ball-param", "w");
        fprintf(f, "%d %d %d %d %d %d %d\n",
                hue_start_level, hue_stop_level,
                sat_start_level, sat_stop_level,
                lum_start_level, lum_stop_level,
                erode_level);
        fclose(f);
      }
      printf("Settings saved.\n");
      break;
    default:
      // ignore other keys.
      break;
    }
    ++iteration;
  }
  cvReleaseCapture(&capture);
}

