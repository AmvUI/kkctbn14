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

namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"
 int iBright = 100;
 int defaultBright = -100;
 int iContrast = 1;

 createTrackbar("Contrast", "Control", &iContrast, 5);//Value (0 - 200)
 createTrackbar("Brightness", "Control", &iBright, 200);//Value (0 - 200)

    while (true)
    {
	
        Mat image, gray;
        
        bool bSuccess = cap.read(image); // read a new frame from video



         if (!bSuccess) //if not success, break loop
        {
             cout << "Cannot read a frame from video stream" << endl;
             break;
        }
    Mat contours;
    Mat gray_image;

    vector<cv::Mat> channels;
	Mat hsv;
    image.convertTo(image, -1, 1, defaultBright + iBright); //Loweer Brightness by 75
    image.convertTo(image, -1, iContrast, 0); //increase the contrast by 2

	cvtColor( image, hsv, CV_RGB2HSV );
	split(hsv, channels);
	gray_image = channels[0];
    Canny(image,contours,10,350);

    namedWindow("Image");
    imshow("Image",image);

    namedWindow("Gray");
    imshow("Gray",gray_image);

    namedWindow("Canny");
    imshow("Canny",contours);
        if (waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
       {
			cout << "esc key is pressed by user" << endl;
            break; 
       }
	}    
	return 0;
	
}
