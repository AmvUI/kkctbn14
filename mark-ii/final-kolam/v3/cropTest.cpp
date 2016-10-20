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
using namespace std;
char key;
int main()
{
    cvNamedWindow("Camera_Output", 1);    //Create window

    CvCapture* capture = cvCaptureFromCAM(0);  //Capture using camera 1 connected to system
    cvSetCaptureProperty( capture, CV_CAP_PROP_FRAME_WIDTH, 640 );
    cvSetCaptureProperty( capture, CV_CAP_PROP_FRAME_HEIGHT, 480 );

    while(1){ //Create loop for live streaming

        IplImage* framein = cvQueryFrame(capture); //Create image frames from capture

        /* sets the Region of Interest  - rectangle area has to be __INSIDE__ the image */
        // CvRect cvRect(int x, int y, int width, int height)	
		//	x – x-coordinate of the top-left corner.
		//	y – y-coordinate of the top-left corner (sometimes bottom-left corner).
        cvSetImageROI(framein, cvRect(0, 240, 640, 100));

        /* create destination image  - cvGetSize will return the width and the height of ROI */
        IplImage *frameout = cvCreateImage(cvGetSize(framein),  framein->depth, framein->nChannels);

        /* copy subimage */
        cvCopy(framein, frameout, NULL);

        /* always reset the Region of Interest */
        cvResetImageROI(framein);

        cvShowImage("Camera_Output", frameout);   //Show image frames on created window

        key = cvWaitKey(10);     //Capture Keyboard stroke
        if (char(key) == 27){
            break;      //ESC key loop will break.
        }
    }

    cvReleaseCapture(&capture); //Release capture.
    cvDestroyWindow("Camera_Output"); //Destroy Window
    return 0;
}
