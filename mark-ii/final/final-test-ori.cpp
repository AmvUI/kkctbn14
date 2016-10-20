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

// ###################################### DECLARE VARIABLE ################################################ //
char bufferRead[256];
char bufferWrite[3];
int intRead; 
int fd;
int menuChoice;
using namespace cv;
using namespace std;
// ###################################### DECLARE VARIABLE ################################################ //

// ###################################### Serial Communication ################################################ //

//serialStart(Lokasi Port, Baud Rate-nya, Besar Data yang ingin diRead untuk serialRead)
void serialStart(const char* portname, speed_t baud, int data)
{
/* Open the file descriptor in non-blocking mode */
 fd = open(portname, O_RDWR | O_NOCTTY);
 
/* Set up the control structure */
 struct termios toptions;
 
 /* Get currently set options for the tty */
 tcgetattr(fd, &toptions);
 
/* Set custom options */
 
/* 9600 baud */
 cfsetispeed(&toptions, baud);
 cfsetospeed(&toptions, baud);
 /* 8 bits, no parity, no stop bits */
 toptions.c_cflag &= ~PARENB;
 toptions.c_cflag &= ~CSTOPB;
 toptions.c_cflag &= ~CSIZE;
 toptions.c_cflag |= CS8;
 /* no hardware flow control */
 toptions.c_cflag &= ~CRTSCTS;
 /* enable receiver, ignore status lines */
 toptions.c_cflag |= CREAD | CLOCAL;
 /* disable input/output flow control, disable restart chars */
 toptions.c_iflag &= ~(IXON | IXOFF | IXANY);
 /* disable canonical input, disable echo,
 disable visually erase chars,
 disable terminal-generated signals */
 toptions.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);
 /* disable output processing */
 toptions.c_oflag &= ~OPOST;
 
/* wait for 24 characters to come in before read returns */
toptions.c_cc[VMIN] = data;
 /* no minimum time to wait before read returns */
 toptions.c_cc[VTIME] = 0;
 
/* commit the options */
 tcsetattr(fd, TCSANOW, &toptions);
 
/* Wait for the Arduino to reset */
 usleep(1000*1000);
 /* Flush anything already in the serial buffer */
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

void serialWrite(int dataOut, int dataWrite){
sprintf(bufferWrite, "%d", dataOut);
write(fd, bufferWrite, dataWrite);
}

// ###################################### Serial Communication ################################################ //


//-------- PID parameters --------

// See http://en.wikipedia.org/wiki/PID_controller

// These values must be chosen CAREFULLY. The strategy to find good
// values is to set `ci' and `cd' to 0.0, then try to find a value of
// `cp' that works the best (without too much oscillation) then, from
// that, lower `cp' and increase `cd' until the system is able to
// stalibilize more quickly. Increase `ci' if the system take time to
// move to the target position. There are more complexes method to
// find the "right" values.

static double cp = 0.15;
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



 int main( int argc, char** argv )
 {
	
	// #################### Serial ############### //
	serialStart("/dev/ttyACM0", B9600, 3);
	// #################### Serial ############### //
	
	
	
 
	double              last_time = 0.0; // last time we updated PID
	int                 last_known_x = 320; // last known position of the target.
	double              last_error = 0.0;
	double              i = 0.0; // integral term (here because it is accumulating)
	int                 last_sent_value = -1;
	int                 target = 320; // posisi target
  
	
	
	VideoCapture cap(0); //capture the video from webcam

    if ( !cap.isOpened() )  // if not success, exit program
    {
         cout << "Cannot open the web cam" << endl;
         return -1;
    }

    namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"

  int iLowH = 0;
 int iHighH = 9;

  int iLowS = 28; 
 int iHighS = 255;

  int iLowV = 95;
 int iHighV = 255;
 
 int iBright = 200;
 int iContrast = 1;
 int defaultBright = -200;

  //Create trackbars in "Control" window
 createTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
 createTrackbar("HighH", "Control", &iHighH, 179);

  createTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
 createTrackbar("HighS", "Control", &iHighS, 255);

  createTrackbar("LowV", "Control", &iLowV, 255);//Value (0 - 255)
 createTrackbar("HighV", "Control", &iHighV, 255);
 
 
 createTrackbar("Brightness", "Control", &iBright, 300);//Value (0 - 200)
 createTrackbar("Contrast", "Control", &iContrast, 5);//Value (0 - 200)

  int iLastX = -1; 
 int iLastY = -1;

  //Capture a temporary image from the camera
 Mat imgTmp;
 cap.read(imgTmp); 

  //Create a black image with the size as the camera output
 Mat imgLines = Mat::zeros( imgTmp.size(), CV_8UC3 );;


    while (true)
    {
	
        Mat imgOriginal;

        bool bSuccess = cap.read(imgOriginal); // read a new frame from video



         if (!bSuccess) //if not success, break loop
        {
             cout << "Cannot read a frame from video stream" << endl;
             break;
        }

    Mat imgHSV;


	   vector<Mat> channels; 
       Mat img_hist_equalized;

       cvtColor(imgOriginal, img_hist_equalized, CV_BGR2YCrCb); //change the color image from BGR to YCrCb format

       split(img_hist_equalized,channels); //split the image into channels

       equalizeHist(channels[0], channels[0]); //equalize histogram on the 1st channel (Y)

      merge(channels,img_hist_equalized); //merge 3 channels including the modified 1st channel into one image

      cvtColor(img_hist_equalized, img_hist_equalized, CV_YCrCb2BGR);
      imgOriginal = img_hist_equalized;
      
      imgOriginal.convertTo(imgOriginal, -1, 1, defaultBright + iBright); //Loweer Brightness by 75
      imgOriginal.convertTo(imgOriginal, -1, iContrast, 0); //increase the contrast by 2


   cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
  Mat imgThresholded;

   inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image
      
  //morphological opening (removes small objects from the foreground)
  erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(6, 6)) );
  dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 

  erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
   //morphological closing (removes small holes from the foreground)
  dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(8, 8)) );

   //Calculate the moments of the thresholded image
  Moments oMoments = moments(imgThresholded);

   double dM01 = oMoments.m01;
  double dM10 = oMoments.m10;
  double dArea = oMoments.m00;
  
  

   // if the area <= 10000, I consider that the there are no object in the image and it's because of the noise, the area is not zero 
  if (dArea > 100000)
  {
   //calculate the position of the ball
   int posX = dM10 / dArea;
   int posY = dM01 / dArea;        
    
        
   if (iLastX >= 0 && iLastY >= 0 && posX >= 0 && posY >= 0)
   {
	   
	   
	
	
    //Draw a red line from the previous point to the current point
    line(imgLines, Point(posX, posY), Point(iLastX, iLastY), Scalar(0,0,255), 2);
		
		
	    
	      //-------- PID processing --------

    // If the object is out of the window, use last known position to
    // find it.
    if (posX < 0 || posX > 640) {
      posX = 2.5 * (last_known_x - 320) + 320;
    } else {
      last_known_x = posX;
    }

    double              time = getTime();
    double              dt = time - last_time;
    if (last_time == 0.0){dt = 1.0;}
    last_time = time;

    double              error = posX - target;
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
    else if (pid > 99){pid = 99;}
	
	serialWrite(pid,3);
	printf("position (%d,%d) pid (%f)\n", posX, posY, pid);
	    
	    
	    
	    
   }

   iLastX = posX;
   iLastY = posY;
  
  }

   imshow("Thresholded Image", imgThresholded); //show the thresholded image
   imshow("Balancing", img_hist_equalized); //show the thresholded image
   imgOriginal = imgOriginal + imgLines;
   imshow("Original", imgOriginal); //show the original image

        if (waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
       {
			cout << "esc key is pressed by user" << endl;
            break; 
       }
    }
    
    
	return 0;
	
}
