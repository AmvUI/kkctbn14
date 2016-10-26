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
#include <sys/time.h>
#include <string.h>

using namespace cv;
using namespace std;

// ###################################### DECLARE VARIABLE ################################################ //

const char*      portACM         = "/dev/ttyACM0";
int				 videoCam		 = 0;
char 			 bufferRead[256];
char 			 bufferWrite[42];
int 			 intRead;
int 			 fd;
int              tcolor         = 0; // target color - Used to switch to predefined hue levels.
double           last_time      = 0.0; // last time we updated PID
int              last_known_x   = 320; // last known position of the target.
double           last_error     = 0.0;
double           i              = 0.0; // integral term (here because it is accumulating)
int              last_sent_value= -1;

int	    		 rightBaseSpeed = 850 ;
int	    		 leftBaseSpeed  = 850 ;
int              turnPoint      = 1500;
int              breakPoint     = 780;

int              hue_level_start= 111;
int              hue_level_stop = 8;
int              sat_level 		= 149;  //aslinya 15
int              target 		= 320; // posisi target

int              motorKiri 		= 0;
int              motorKanan 	= 0;
int              kiriMundur		= 0 ;
int              kiriBelok 		= 80;
int              kananMundur	= 180 ;
int              kananBelok 	= 10;

static double    cp             = 2;
static double    ci             = 0.0;
static double    cd             = 0.02;
char 			 movementStat[20];
char			 statKiriPatah[20] = "Kiri Patah";
char			 statKananPatah[20] = "Kanan Patah";
char			 statNormalPID[20] = "Normal PID";
char			 statKiriREM[20] = "Kiri REM";
char			 statKananREM[20] = "Kanan REM";

// ###################################### DECLARE VARIABLE ################################################ //


//#################################### MOTOR MOVE #########################################//
void maju(){
  kiriMundur 	= 90; //  0 maju ; 90 mundur
  kiriBelok 	= 80; // < 100; 60; > 30 
  kananMundur 	= 180; // 180 maju ; 120 mundur
  kananBelok 	= 10; //  <30 ; 10 ; 0>
}

void rem(int state){
  if(state == 1){
  kiriMundur 	= 20; //  5 maju ; 70 kiri mundur
  kiriBelok 	= 80; // < 100; 60; > 30 kiri belok
    }

  else if(state == 2){
  kananMundur 	= 120; // 120 mundur ; 180 kanan maju
  kananBelok 	= 10; //  <45 ; 25 ; 0>
    }

  else {
  kiriMundur 	= 90; //  5 maju ; 70 kiri mundur
  kiriBelok 	= 80; // < 100; 60; > 30 kiri belok
  kananMundur 	= 120; // 120 mundur ; 180 kanan maju
  kananBelok 	= 10; //  <45 ; 25 ; 0>
    }
}

void belokKiri(){
  kiriMundur 	= 0; //  < 100; 60; > 30 kiri belok
  kiriBelok 	= 40; // 5 maju ; 70 kiri mundur
  kananMundur 	= 180; // 120 mundur ; 180 kanan maju
  kananBelok 	= 30; //  <45 ; 25 ; 0>

}

void belokKanan(){
  kiriMundur 	= 90; //  < 100; 60; > 30 kiri belok
  kiriBelok 	= 40; // 5 maju ; 70 kiri mundur
  kananMundur 	= 130; // 120 mundur ; 180 kanan maju
  kananBelok 	= 30; //  <45 ; 25 ; 0>
}

//#################################### MOTOR MOVE #########################################//

// ###################################### Serial Communication ################################################ //

//serialStart(Lokasi Port, Baud Rate-nya, Besar Data yang ingin diRead untuk serialRead)
void serialStart(const char* portname, speed_t baud, int data)
{
 fd = open(portname, O_RDWR | O_NOCTTY);
 struct termios toptions;
 tcgetattr(fd, &toptions);
 cfsetispeed(&toptions, baud);
 cfsetospeed(&toptions, baud);
 toptions.c_cflag &= ~PARENB;
 toptions.c_cflag &= ~CSTOPB;
 toptions.c_cflag &= ~CSIZE;
 toptions.c_cflag |= CS8;
 toptions.c_cflag &= ~CRTSCTS;
 toptions.c_cflag |= CREAD | CLOCAL;
 toptions.c_iflag &= ~(IXON | IXOFF | IXANY);
 toptions.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);
 toptions.c_oflag &= ~OPOST;
 toptions.c_cc[VMIN] = data;
 toptions.c_cc[VTIME] = 0;
 tcsetattr(fd, TCSANOW, &toptions);
 usleep(1000*1000);
 tcflush(fd, TCIFLUSH);
 }

//Data tersimpan di BufferRead, untuk data yang udah jadi integer bisa pakai intRead
//serialRead(Besar Data yang akan diread)
//Contoh Test : serialRead(3); printf("%d   %d\n", intRead, intRead*2 );
void serialRead(int dataRead){
read(fd, bufferRead, dataRead);
intRead = atoi(bufferRead);
 }

//Ada bit 0 diakhir setiap ngirim data. Buat filter di mikon untuk nge-abaikaan 0
//serialWrite(Integer yang ingin dikirim, Besar Bilangan Integernya)
//Contoh : serialWrite(123,3);}


void serialWrite(int dataOut1,int dataOut2,int dataOut3,int dataOut4,int dataOut5,int dataOut6, int dataWrite){
sprintf(bufferWrite, "%d,%d,%d,%d,%d,%d,%d,%d,",8888,(dataOut1)+10000,(dataOut2)+20000,(dataOut3)+3000,(dataOut4)+4000,(dataOut5)+5000,(dataOut6)+6000,9999);
write(fd, bufferWrite, dataWrite);
}

// ###################################### Serial Communication ################################################ //



static double gettime()
{
  struct timeval tv;
  gettimeofday(&tv, 0);
  return tv.tv_sec + tv.tv_usec / 1e6;
}

int main()
{
  CvCapture*          capture = cvCreateCameraCapture(videoCam);
  serialStart(portACM, B57600, 42);
  //cvNamedWindow("RGB", CV_WINDOW_AUTOSIZE);
  //cvMoveWindow("RGB", 0, 0);
  cvNamedWindow("Mask", CV_WINDOW_AUTOSIZE);
  cvMoveWindow("Mask", 0, 505);
  cvNamedWindow("Settings", 0);
  cvMoveWindow("Settings", 652, 505);
  cvCreateTrackbar("Hue level (start)", "Settings", &hue_level_start, 255, 0); //103
  cvCreateTrackbar("Hue level (stop)", "Settings", &hue_level_stop, 255, 0); //113
  cvCreateTrackbar("Saturation level", "Settings", &sat_level, 255, 0); //136
  cvCreateTrackbar("Target", "Settings", &target, 640, 0);
  cvNamedWindow("Track", CV_WINDOW_AUTOSIZE);
  cvMoveWindow("Track", 646, 0);
  IplImage*           chan1 = 0;
  IplImage*           chan2 = 0;
  IplImage*           chan3 = 0;
  IplImage*           hsv = 0;
  IplImage*           monitor = 0;
  IplImage*           result = 0;
  for (int n = 0;; ++n) {
    IplImage*           frame = cvQueryFrame(capture);
    if (frame == 0) break;
    //cvShowImage("RGB", frame);
    if (hsv == 0) {
	  hsv     = cvCreateImage(cvGetSize(frame), 8, 3); 
      chan1   = cvCreateImage(cvGetSize(frame), 8, 1); 
      chan2   = cvCreateImage(cvGetSize(frame), 8, 1);
      chan3   = cvCreateImage(cvGetSize(frame), 8, 1);
      monitor = cvCreateImage(cvGetSize(frame), 8, 3);
      result  = cvCreateImage(cvGetSize(frame), 8, 3);
    }
    cvCvtColor(frame, hsv, CV_BGR2HSV); 
    cvSplit(hsv, chan1, chan2, 0, 0);
    if (hue_level_start <= hue_level_stop) {
      cvInRangeS(chan1, cvScalar(hue_level_start), cvScalar(hue_level_stop), chan1);
    } else {
      cvInRangeS(chan1, cvScalar(hue_level_stop), cvScalar(hue_level_start), chan1);
      cvSubRS(chan1, cvScalar(255), chan1);
    }
    cvCmpS(chan2, sat_level, chan2, CV_CMP_GT); 
    cvAnd(chan1, chan2, chan3);
    cvErode(chan3, chan3, 0, 2);
	CvMoments           moments;
    cvMoments(chan3, &moments, 1);
    int                 r = sqrt(moments.m00);
    int                 x = moments.m10/moments.m00;
    int                 y = moments.m01/moments.m00;

    cvConvertScale(frame, monitor, .3, 0); // faded out input
    cvSet(monitor, cvScalar(255, 0, 0), chan1); // blue overlay
    cvSet(monitor, cvScalar(0, 255, 0), chan2); // green overlay
    cvSet(monitor, cvScalar(0, 0, 255), chan3); // red overlay
    if (x > 0 && y > 0) {
      cvCircle(monitor, cvPoint(x, y), r, cvScalar(0, 0, 0), 4, CV_AA, 0);
      cvCircle(monitor, cvPoint(x, y), r, cvScalar(255, 255, 255), 2, CV_AA, 0);
    }
    //cvShowImage("Mask", monitor);

    //-------- Tracking state --------

    cvCopy(frame, result, 0); // input image
    cvLine(result, cvPoint(target - 60, 0), cvPoint(target - 60, 480), cvScalar(0, 0, 0), 2); // 480=panjang garis;
    // cvscalar(biru, hijau, Merah),tebal
    cvLine(result, cvPoint(target, 0), cvPoint(target, 480), cvScalar(0, 0, 255), 3);
    cvLine(result, cvPoint(target + 60, 0), cvPoint(target + 60, 480), cvScalar(0, 0, 0), 2); // 60=lebar garis item
//    cvLine(result, cvPoint(0, target + 60), cvPoint(300, target + 60), cvScalar(255, 0, 0), 2);
    if (x > 0 && y > 0) {
      cvCircle(result, cvPoint(x, y), 10, cvScalar(0, 0, 0), 6, CV_AA, 0);
      cvCircle(result, cvPoint(x, y), 10, cvScalar(0, 255, 255), 4, CV_AA, 0);
    }
    //cvShowImage("Track", result);

    //-------- Handle keyboard events --------

    int key = cvWaitKey(33);
    if (key == 27) break;
    switch (key) {
    case 'r':
      // Reset the current position. This is used to check how fast
      // the system can return to the correct position.
      break;
    case 't':
      // Quickly switch to a different tracking color (red or blue)
      tcolor = 1 - tcolor;
      if (tcolor == 0) {
        cvSetTrackbarPos("Hue level (start)", "Settings", 0);
        cvSetTrackbarPos("Hue level (stop)", "Settings", 12);
      } else {
        cvSetTrackbarPos("Hue level (start)", "Settings", 109);
        cvSetTrackbarPos("Hue level (stop)", "Settings", 116);
      }
      break;
    default:
      // ignore other keys.
      break;
    }

    //-------- PID processing --------

    // If the object is out of the window, use last known position to
    // find it.
    if (x < 0 || x > 640) {
      x = 2.5 * (last_known_x - 320) + 320;
    } else {
      last_known_x = x;
    }

    double              time = gettime();
    double              dt = time - last_time;
    if (last_time == 0.0)
      dt = 1.0;
    last_time = time;

    // the error we want to correct
    double              error = (x - target);

    // the proportional term
    double              p = error * cp;

    // update the integral term
    i += error * dt * ci;

    // Clamp integral term
    if (i > 30.0)
      i = 30.0;
    else if (i < -30.0)
      i = -30.0;

    // the derivative term
    double              d = (error - last_error) / dt * cd;
    last_error = error;

    // the PID value
    double              pid = p + i + d;


        // ######## PID Value in Camera Perspective #####
        // ##############################################
        // #                                            #
        // #                                            #
        // #             - <=== 0 ===> +                #
        // #                                            #
        // #                                            #
        // ##############################################

      //BELOK KANAN - jika pid berlebih
      //kalo motorkanan lebih kecil dengan nilai yang jauh maka akan menjadi belokKanan
      if (pid > turnPoint-rightBaseSpeed){
        belokKanan();
        motorKiri	= leftBaseSpeed+pid;
        motorKanan	= rightBaseSpeed+pid;
        strcpy(movementStat, statKananPatah);
      }

      //BELOK KIRI - jika pid berlebih
      //kalo motorkiri lebih kecil dengan nilai yang jauh maka akan menjadi belokKiri
      else if (pid < (0 - (turnPoint-rightBaseSpeed))){
        belokKiri();
        motorKiri = leftBaseSpeed+(0-pid);
        motorKanan = rightBaseSpeed+(0-pid);
        strcpy(movementStat, statKiriPatah);
      }

      // TETEP NGIKUTIN TARGET
      //belok-belok pid yang biasa
      else {
         maju();
	 motorKanan = rightBaseSpeed - pid;
         motorKiri = leftBaseSpeed + pid;
         strcpy(movementStat, statNormalPID);
       }


       //kalo mau BELOK KIRI yang kanan di kecilin
       //kalo dikurangin hinngga menjadi dibawah 780 atau breakPoint
       //maka motor 1 jadi mundur
       if (motorKiri < breakPoint){
         maju();
	 rem(1);
         motorKiri = (breakPoint-motorKiri) + breakPoint;
		 strcpy(movementStat, statKiriREM);
       }

       //kalo mau BELOK KANAN yang kanan di kecilin
       //kalo dikuranginn hinngga menjadi dibawah 780 atau breakPoint
       //maka motor 2 jadi mundur
       if (motorKanan < breakPoint){
         maju();
	 rem(2);
         motorKanan = (breakPoint-motorKanan) + breakPoint;
         strcpy(movementStat, statKananREM);
       }
		
		 usleep(185000);
       serialWrite(motorKiri, motorKanan, kiriMundur, kiriBelok, kananMundur, kananBelok ,42);
       printf("Throttle : %s %f %d %d %d %d %d %d\n",movementStat, pid,  motorKiri, motorKanan,kiriMundur, kiriBelok, kananMundur, kananBelok);


    //-------- Send the position to Arduino --------

   }
  cvReleaseCapture(&capture);
}

