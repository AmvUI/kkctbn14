<br />
/*****************************************************************************************<br />
*  Name    : Fast object tracking using the OpenCV library                               *<br />
*  Author  : Lior Chen &lt;chen.lior@gmail.com&gt;                                             *<br />
*  Notice  : Copyright (c) Jun 2010, Lior Chen, All Rights Reserved                      *<br />
*          :                                                                             *<br />
*  Site    : http://www.lirtex.com                                                       *<br />
*  WebPage : http://www.lirtex.com/robotics/fast-object-tracking-robot-computer-vision   *<br />
*          :                                                                             *<br />
*  Version : 1.0                                                                         *<br />
*  Notes   : By default this code will open the first connected camera.                  *<br />
*          : In order to change to another camera, change                                *<br />
*          : CvCapture* capture = cvCaptureFromCAM( 0 ); to 1,2,3, etc.                  *<br />
*          : Also, the code is currently configured to tracking RED objects.             *<br />
*          : This can be changed by changing the hsv_min and hsv_max vectors             *<br />
*          :                                                                             *<br />
*  License : This program is free software: you can redistribute it and/or modify        *<br />
*          : it under the terms of the GNU General Public License as published by        *<br />
*          : the Free Software Foundation, either version 3 of the License, or           *<br />
*          : (at your option) any later version.                                         *<br />
*          :                                                                             *<br />
*          : This program is distributed in the hope that it will be useful,             *<br />
*          : but WITHOUT ANY WARRANTY; without even the implied warranty of              *<br />
*          : MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the               *<br />
*          : GNU General Public License for more details.                                *<br />
*          :                                                                             *<br />
*          : You should have received a copy of the GNU General Public License           *<br />
*          : along with this program.  If not, see &lt;http://www.gnu.org/licenses/&gt;        *<br />
******************************************************************************************/</p>
<p>#include &lt;opencv/cvaux.h&gt;<br />
#include &lt;opencv/highgui.h&gt;<br />
#include &lt;opencv/cxcore.h&gt;<br />
#include &lt;stdio.h&gt;</p>
<p>#include &lt;stdio.h&gt;<br />
#include &lt;stdlib.h&gt;<br />
#include &lt;string.h&gt;<br />
#include &lt;assert.h&gt;<br />
#include &lt;math.h&gt;<br />
#include &lt;float.h&gt;<br />
#include &lt;limits.h&gt;<br />
#include &lt;time.h&gt;<br />
#include &lt;ctype.h&gt;</p>
<p>int main(int argc, char* argv[])<br />
{</p>
<p>    // Default capture size - 640x480<br />
    CvSize size = cvSize(640,480);</p>
<p>    // Open capture device. 0 is /dev/video0, 1 is /dev/video1, etc.<br />
    CvCapture* capture = cvCaptureFromCAM( 0 );<br />
    if( !capture )<br />
    {<br />
            fprintf( stderr, &quot;ERROR: capture is NULL \n&quot; );<br />
            getchar();<br />
            return -1;<br />
    }</p>
<p>    // Create a window in which the captured images will be presented<br />
    cvNamedWindow( &quot;Camera&quot;, CV_WINDOW_AUTOSIZE );<br />
    cvNamedWindow( &quot;HSV&quot;, CV_WINDOW_AUTOSIZE );<br />
    cvNamedWindow( &quot;EdgeDetection&quot;, CV_WINDOW_AUTOSIZE );</p>
<p>    // Detect a red ball<br />
    CvScalar hsv_min = cvScalar(150, 84, 130, 0);<br />
    CvScalar hsv_max = cvScalar(358, 256, 255, 0);</p>
<p>    IplImage *  hsv_frame    = cvCreateImage(size, IPL_DEPTH_8U, 3);<br />
    IplImage*  thresholded   = cvCreateImage(size, IPL_DEPTH_8U, 1);</p>
<p>    while( 1 )<br />
    {<br />
        // Get one frame<br />
        IplImage* frame = cvQueryFrame( capture );<br />
        if( !frame )<br />
        {<br />
                fprintf( stderr, &quot;ERROR: frame is null...\n&quot; );<br />
                getchar();<br />
                break;<br />
        }</p>
<p>        // Covert color space to HSV as it is much easier to filter colors in the HSV color-space.<br />
        cvCvtColor(frame, hsv_frame, CV_BGR2HSV);<br />
        // Filter out colors which are out of range.<br />
        cvInRangeS(hsv_frame, hsv_min, hsv_max, thresholded);</p>
<p>        // Memory for hough circles<br />
        CvMemStorage* storage = cvCreateMemStorage(0);<br />
        // hough detector works better with some smoothing of the image<br />
        cvSmooth( thresholded, thresholded, CV_GAUSSIAN, 9, 9 );<br />
        CvSeq* circles = cvHoughCircles(thresholded, storage, CV_HOUGH_GRADIENT, 2,<br />
                                        thresholded-&gt;height/4, 100, 50, 10, 400);</p>
<p>        for (int i = 0; i &lt; circles-&gt;total; i++)<br />
        {<br />
            float* p = (float*)cvGetSeqElem( circles, i );<br />
            printf(&quot;Ball! x=%f y=%f r=%f\n\r&quot;,p[0],p[1],p[2] );<br />
            cvCircle( frame, cvPoint(cvRound(p[0]),cvRound(p[1])),<br />
                                    3, CV_RGB(0,255,0), -1, 8, 0 );<br />
            cvCircle( frame, cvPoint(cvRound(p[0]),cvRound(p[1])),<br />
                                    cvRound(p[2]), CV_RGB(255,0,0), 3, 8, 0 );<br />
        }</p>
<p>        cvShowImage( &quot;Camera&quot;, frame ); // Original stream with detected ball overlay<br />
        cvShowImage( &quot;HSV&quot;, hsv_frame); // Original stream in the HSV color space<br />
        cvShowImage( &quot;After Color Filtering&quot;, thresholded ); // The stream after color filtering</p>
<p>        cvReleaseMemStorage(&amp;storage);</p>
<p>        // Do not release the frame!</p>
<p>        //If ESC key pressed, Key=0x10001B under OpenCV 0.9.7(linux version),<br />
        //remove higher bits using AND operator<br />
        if( (cvWaitKey(10) &amp; 255) == 27 ) break;<br />
    }</p>
<p>     // Release the capture device housekeeping<br />
     cvReleaseCapture( &amp;capture );<br />
     cvDestroyWindow( &quot;mywindow&quot; );<br />
     return 0;<br />
   }<br />