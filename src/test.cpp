#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>

using namespace cv;
using namespace std;

int main (int argc, char *argv[]) {
    VideoCapture cap(0);

    if (!cap.isOpened()) {
        cout << "ERROR opening stream" << endl;
        return -1;
    }

    double dWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH);
    double dHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT);

    cout << "Frame size : " << dWidth << " x " << dHeight << endl;

    
    namedWindow("MyVideo",CV_WINDOW_AUTOSIZE); //create a window called "MyVideo"

    while(1) {
        Mat frame;

        // break if a frame cannot be read.
        if ( !cap.read(frame) ) {
            cout << "Cannot read a frame from video stream" << endl;
            break;
        }

        imshow("MyVideo", frame); //show the frame in "MyVideo" window
        
        // break if the ESC held for 30ms
        if (waitKey(30) == 27) {
            cout << "esc key is pressed by user" << endl;
            break; 
        }
                                            }
        return 0;
}

