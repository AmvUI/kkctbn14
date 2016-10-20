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
 
/* My Arduino is on /dev/ttyACM0 */
char bufferRead[256];
char bufferWrite[3];
int intRead; 
int fd;

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

using namespace cv;
using namespace std;

 int main( int argc, char** argv )
 {
	serialStart("/dev/ttyACM0", B9600, 3);
	VideoCapture cap(1); //capture the video from webcam 1
	VideoCapture cap2(0); //capture the video from webcam 2

    if ( !cap.isOpened() )  // if not success, exit program
    {
         cout << "Cannot open the web cam" << endl;
         return -1;
    }
    
    if ( !cap2.isOpened() )  // if not success, exit program
    {
         cout << "Cannot open the web cam" << endl;
         return -1;
    }

// ############################### TO CONTROL HSV #################################### //

 namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"

 int iLowH = 0;
 int iHighH = 10;

 int iLowS = 034; 
 int iHighS = 255;

 int iLowV = 137;
 int iHighV = 255;

  //Create trackbars in "Control" window
 createTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
 createTrackbar("HighH", "Control", &iHighH, 179);

  createTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
 createTrackbar("HighS", "Control", &iHighS, 255);

  createTrackbar("LowV", "Control", &iLowV, 255);//Value (0 - 255)
 createTrackbar("HighV", "Control", &iHighV, 255);

// ############################### TO CONTROL HSV END #################################### //

  int iLastX = -1; 
  int iLastY = -1;
  int iLastX2 = -1; 
  int iLastY2 = -1;

 //Capture a temporary image from the camera 1
 Mat imgTmp;
 cap.read(imgTmp); 

 //Capture a temporary image from the camera 2
 Mat imgTmp2;
 cap2.read(imgTmp2); 


 //Create a black image with the size as the camera output 1
 Mat imgLines = Mat::zeros( imgTmp.size(), CV_8UC3 );;
 
 //Create a black image with the size as the camera output 2
 Mat imgLines2 = Mat::zeros( imgTmp2.size(), CV_8UC3 );;

    while (true)
    {
        Mat imgOriginal;
        Mat imgOriginal2;

        bool bSuccess = cap.read(imgOriginal); // read a new frame from video 1
		bool bSuccess2 = cap2.read(imgOriginal2); // read a new frame from video 2


         if (!bSuccess) //if not success, break loop
        {
             cout << "Cannot read a frame from video stream" << endl;
             break;
        }

		if (!bSuccess2) //if not success, break loop
        {
             cout << "Cannot read a frame from video stream" << endl;
             break;
        }

    Mat imgHSV;
    Mat imgHSV2;

   cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
   cvtColor(imgOriginal2, imgHSV2, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
 
  Mat imgThresholded;
  Mat imgThresholded2;

 // ############################################ THRESHOLD THE IMAGE ################################################### //
   inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image 1
   inRange(imgHSV2, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded2); //Threshold the image 2
 // ######################################### THRESHOLD THE IMAGE  END ################################################# //      
      
 // ################################################# REMOVE NOISE ###################################################### //
  //morphological opening (removes small objects from the foreground) - CAMERA 1
  erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
  dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
	
  erode(imgThresholded2, imgThresholded2, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
  dilate( imgThresholded2, imgThresholded2, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 


   //morphological closing (removes small holes from the foreground) - CAMERA 2
  dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
  erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
  
  
  dilate( imgThresholded2, imgThresholded2, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
  erode(imgThresholded2, imgThresholded2, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
// ################################################# REMOVE NOISE END ################################################### //

   //Calculate the moments of the thresholded image 1
  Moments oMoments = moments(imgThresholded);
 
   //Calculate the moments of the thresholded image 2
  Moments oMoments2 = moments(imgThresholded2);
  
  double dM01 = oMoments.m01;
  double dM10 = oMoments.m10;
  double dArea = oMoments.m00;
	
  double dM012 = oMoments2.m01;
  double dM102 = oMoments2.m10;
  double dArea2 = oMoments2.m00;
  
  

   // if the area <= 10000, I consider that the there are no object in the image and it's because of the noise, the area is not zero 
  if (dArea > 10000 | dArea2 > 10000)
  {
   //calculate the position of the ball
   int posX = dM10 / dArea;
   int posY = dM01 / dArea; 
   
   int posX2 = dM102 / dArea2;
   int posY2 = dM012 / dArea2; 
          
    
        
   if (iLastX >= 0 && iLastY >= 0 && posX >= 0 && posY >= 0)
   {
    //Draw a red line from the previous point to the current point
    line(imgLines, Point(posX, posY), Point(iLastX, iLastY), Scalar(0,0,255), 2);
		
		serialWrite(posX,3);
	    printf("position 1 (%d,%d)\n", posX, posY);
   }
   
   if (iLastX2 >= 0 && iLastY2 >= 0 && posX2 >= 0 && posY2 >= 0)
   {
    //Draw a red line from the previous point to the current point
    line(imgLines2, Point(posX2, posY2), Point(iLastX2, iLastY2), Scalar(0,0,255), 2);
		
		serialWrite(posX2,3);
	    printf("position 2 (%d,%d)\n", posX2, posY2);
   }

   iLastX = posX;
   iLastY = posY;

   iLastX2 = posX2;
   iLastY2 = posY2;
    
  
  }

   
   
   imshow("Thresholded Image", imgThresholded); //show the thresholded image 
   imgOriginal = imgOriginal + imgLines;
   imshow("Original", imgOriginal); //show the original image


   imshow("Thresholded Image 2", imgThresholded2); //show the thresholded image
   imgOriginal2 = imgOriginal2 + imgLines2;
   imshow("Original 2", imgOriginal2); //show the original image

        if (waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
       {
			cout << "esc key is pressed by user" << endl;
            break; 
       }
    }

   return 0;
}
