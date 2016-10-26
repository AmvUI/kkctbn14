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
int              last_known_x   = 500; // last known position of the target.
int		 x	=	500;
double           last_error     = 0.0;
double           i              = 0.0; // integral term (here because it is accumulating)
int              last_sent_value= -1;

int	    		 rightBaseSpeed = 850 ;
int	    		 leftBaseSpeed  = 850 ;
int              turnPoint      = 1200;
int              breakPoint     = 780;

int              hue_level_start= 154;
int              hue_level_stop = 8;
int              sat_level 		= 99;  //aslinya 15
int              target 		= 500; // posisi target

int              motorKiri 		= 0;
int              motorKanan 	= 0;
int              kiriMundur		= 0 ;
int              kiriBelok 		= 80;
int              kananMundur	= 180 ;
int              kananBelok 	= 10;
int				 maxSpeed		= 1300;

static double    cp             = 1;
static double    ci             = 0;
static double    cd             = 0;
char 			 movementStat[20];
char			 statKiriPatah[20] = "Kiri Patah";
char			 statKananPatah[20] = "Kanan Patah";
char			 statNormalPID[20] = "Normal PID";
char			 statKiriREM[20] = "Kiri REM";
char			 statKananREM[20] = "Kanan REM";
int				 nozzleState	= 0;

// ###################################### DECLARE VARIABLE ################################################ //


//#################################### MOTOR MOVE #########################################//

void maju(){
//  kiriMundur 	= 90; //  0 maju ; 90 mundur
//  kiriBelok 	= 80; // < 100; 60; > 30 
//  kananMundur 	= 180; // 180 maju ; 120 mundur
//  kananBelok 	= 10; //  <30 ; 10 ; 0>
nozzleState		=	0;

}
void rem(int nozzleSide){
//  kiriMundur 	= 20; //  5 maju ; 70 kiri mundur
//  kiriBelok 	= 80; // < 100; 60; > 30 kiri belok
//  kananMundur 	= 120; // 120 mundur ; 180 kanan maju
//  kananBelok 	= 10; //  <45 ; 25 ; 0>
if (nozzleSide == 1){
	nozzleState		=	1;
	}

if (nozzleSide == 2){
	nozzleState		=	2;
	}

if (nozzleSide == 3){
	nozzleState		=	3;
	}
}

void belokKiri(){
//  kiriMundur 	= 20; //  < 100; 60; > 30 kiri belok
//  kiriBelok 	= 40; // 5 maju ; 70 kiri mundur
//  kananMundur 	= 180; // 120 mundur ; 180 kanan maju
//  kananBelok 	= 30; //  <45 ; 25 ; 0>
	nozzleState		=	4;
}

void belokKanan(){
//  kiriMundur 	= 90; //  < 100; 60; > 30 kiri belok
//  kiriBelok 	= 40; // 5 maju ; 70 kiri mundur
//  kananMundur 	= 130; // 120 mundur ; 180 kanan maju
//  kananBelok 	= 30; //  <45 ; 25 ; 0>
	nozzleState		=	5;
}

void majuKiri(){
nozzleState		=	6;

}

void majuKanan(){
nozzleState		=	7;

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


void serialWrite(int dataOut1,int dataOut2,int dataOut3, int dataWrite){
//Lihat diarduino, awalnya startFlag emang 8 dan stopFlag 9 namun karena
//ada bug di algoritma untuk ngubah asci jadi int == >  karena num=calc() +1; 
//semua yang hanya berdigit 2 atau 1 jadi ditambah 1
sprintf(bufferWrite, "%d,%d,%d,%d,%d,",7,(dataOut1)+10000,(dataOut2)+20000,(dataOut3)+30,8);
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
  cvSetCaptureProperty( capture, CV_CAP_PROP_FRAME_WIDTH, 640 );
  cvSetCaptureProperty( capture, CV_CAP_PROP_FRAME_HEIGHT, 480 );
  
//  serialStart(portACM, B115200, 19);
  cvNamedWindow("RGB", CV_WINDOW_AUTOSIZE);
  cvMoveWindow("RGB", 0, 0);
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
	
	  IplImage* framein = cvQueryFrame(capture); //Create image frames from capture

	/* sets the Region of Interest  - rectangle area has to be __INSIDE__ the image */
	// CvRect cvRect(int x, int y, int width, int height)	
	//	x – x-coordinate of the top-left corner.
	//	y – y-coordinate of the top-left corner (sometimes bottom-left corner).
	cvSetImageROI(framein, cvRect(0, 370, 640, 100));

	/* create destination image  - cvGetSize will return the width and the height of ROI */
	IplImage *frameout = cvCreateImage(cvGetSize(framein),  framein->depth, framein->nChannels);

	/* copy subimage */
	cvCopy(framein, frameout, NULL);

	/* always reset the Region of Interest */
	cvResetImageROI(framein);
    IplImage*           frame = frameout;
    if (frame == 0) break;
   cvShowImage("RGB", frame);
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
//    cvErode(chan3, chan3, 0, 2);
	CvMoments           moments;
    cvMoments(chan3, &moments, 1);
    int                 r = sqrt(moments.m00);
	x = moments.m10/moments.m00;
    int                 y = moments.m01/moments.m00;

    cvConvertScale(frame, monitor, .3, 0); // faded out input
    cvSet(monitor, cvScalar(255, 0, 0), chan1); // blue overlay
    cvSet(monitor, cvScalar(0, 255, 0), chan2); // green overlay
    cvSet(monitor, cvScalar(0, 0, 255), chan3); // red overlay
    if (x > 0 && y > 0) {
      cvCircle(monitor, cvPoint(x, y), r, cvScalar(0, 0, 0), 4, CV_AA, 0);
      cvCircle(monitor, cvPoint(x, y), r, cvScalar(255, 255, 255), 2, CV_AA, 0);
    }
   cvShowImage("Mask", monitor);

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
    cvShowImage("Track", result);

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
	
	

 if (x < 1 || x > 640) {
      x = last_known_x;
      }
      else { last_known_x = x;}



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
        // #    4     1        6 7             2    5   #
        // ##############################################

      //BELOK KANAN - jika pid berlebih
      //kalo motorkanan lebih kecil dengan nilai yang jauh maka akan menjadi belokKanan
      if (pid > turnPoint-rightBaseSpeed){
        belokKanan(); //5
        motorKiri	= leftBaseSpeed+pid;
        motorKanan	= rightBaseSpeed+pid;
        strcpy(movementStat, statKananPatah);
      }

      //BELOK KIRI - jika pid berlebih
      //kalo motorkiri lebih kecil dengan nilai yang jauh maka akan menjadi belokKiri
      else if (pid < (0 - (turnPoint-rightBaseSpeed))){
        belokKiri(); //4
        motorKiri = leftBaseSpeed+(0-pid);
        motorKanan = rightBaseSpeed+(0-pid);
        strcpy(movementStat, statKiriPatah);
      }

      // TETEP NGIKUTIN TARGET
      //belok-belok pid yang biasa
      else {
		 motorKanan = rightBaseSpeed - pid;
         motorKiri = leftBaseSpeed + pid;
         maju();
         if (motorKanan > motorKiri){
			 majuKiri();
			 }
         if (motorKiri > motorKanan){
			 majuKanan();
			 }
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
		
		
		//Clamp Max Motor Speed
		if (motorKiri > maxSpeed) {motorKiri = maxSpeed;}
		if (motorKanan > maxSpeed) {motorKanan = maxSpeed;}
		// usleep(185000);
//       serialWrite(motorKiri, motorKanan, nozzleState,19);
       printf("Throttle : %s %f %d %d %d %d %d\n",movementStat, pid,x,last_known_x,  motorKiri, motorKanan,nozzleState);

	//	lastnozzleState = nozzleState;
    //-------- Send the position to Arduino --------
		
   }
  cvReleaseCapture(&capture);
}


