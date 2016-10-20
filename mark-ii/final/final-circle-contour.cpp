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

 int main( int argc, char** argv )
 {
// #################### Serial ############### //
	serialStart("/dev/ttyACM0", B9600, 3);
	// #################### Serial ############### //
	
	
	VideoCapture cap(2); //capture the video from webcam

    if ( !cap.isOpened() )  // if not success, exit program
    {
         cout << "Cannot open the web cam" << endl;
         return -1;
    }


    while (true)
    {
	
        Mat src, canny, gray;

        bool bSuccess = cap.read(src); // read a new frame from video



         if (!bSuccess) //if not success, break loop
        {
             cout << "Cannot read a frame from video stream" << endl;
             break;
        }
    
    cvtColor( src, gray, CV_BGR2GRAY );
 
  // Reduce the noise so we avoid false circle detection
  GaussianBlur( gray, gray, Size(9, 9), 2, 2 );

  Canny(src,canny,10,350);

 
  vector<Vec3f> circles;
 
  // Apply the Hough Transform to find the circles
  HoughCircles( canny, circles, CV_HOUGH_GRADIENT, 1, 30, 200, 50, 0, 0 );
 
  // Draw the circles detected
  for( size_t i = 0; i < circles.size(); i++ )
  {
      Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
      int radius = cvRound(circles[i][2]);    
      circle( src, center, 3, Scalar(0,255,0), -1, 8, 0 );// circle center    
      circle( src, center, radius, Scalar(0,0,255), 3, 8, 0 );// circle outline
      cout << "center : " << center << "\nradius : " << radius << endl;
   }
 
  // Show your results
  namedWindow( "Hough Circle Transform Demo", CV_WINDOW_AUTOSIZE );
  imshow( "Hough Circle Transform Demo", src );
 
 namedWindow( "Hough Circle Transform", CV_WINDOW_AUTOSIZE );
  imshow( "Hough Circle Transform", gray );

   namedWindow( "Canny", CV_WINDOW_AUTOSIZE );
  imshow( "Canny", gray );
 
        if (waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
       {
			cout << "esc key is pressed by user" << endl;
            break;
       	}
       	
	}    
	return 0;
	
}
