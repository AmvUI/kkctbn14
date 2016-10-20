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

using namespace cv;
using namespace std;

int main(int argc, char* argv[])
{
     VideoCapture cap(0); // open the video camera no. 0

    if (!cap.isOpened())  // if not success, exit program
    {
        cout << "Cannot open the video cam" << endl;
        return -1;
    }

   double dWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH); //get the width of frames of the video
   double dHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT); //get the height of frames of the video

    cout << "Frame size : " << dWidth << " x " << dHeight << endl;


    namedWindow("Original Video",CV_WINDOW_AUTOSIZE); //create a window called "Original Video"
    namedWindow("Brightness Increased",CV_WINDOW_AUTOSIZE); //create a window called "Brightness Increased"
    namedWindow("Brightness Decreased",CV_WINDOW_AUTOSIZE); //create a window called "Brightness Decreased"

    while(1)
    {
           Mat frame;

           bool bSuccess = cap.read(frame); // read a new frame from video

            if (!bSuccess) //if not success, break loop
           {
                        cout << "Cannot read the frame from video file" << endl;
                       break;
           }

            Mat imgH = frame + Scalar(50, 50, 50); //increase the brightness by 75 units

            Mat imgL = frame + Scalar(-50, -50, -50); //decrease the brightness by 75 units

           imshow("Original Video", frame); //show the frame in "Original Video" window
           imshow("Brightness Increased", imgH); //show the frame of which brightness increased
           imshow("Brightness Decreased", imgL); //show the frame of which brightness decreased

           if (waitKey(30) == 27) //wait for 'esc' key press for 30 ms. If 'esc' key is pressed, break loop
           {
                   cout << "esc key is pressed by user" << endl; 
                   break; 
           }
    }

    return 0;

}
