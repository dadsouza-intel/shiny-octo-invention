/*
 * Author: Daniel D'Souza
 */

#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <iostream>

using namespace cv;
using namespace std;

// create a KF 
#define N_STATES 5          // the number of states. idk
#define N_MEASUREMENTS 2    // the number of old states to consider?
#define N_INPUTS 1          // the number of of inputs per prediction
KalmanFilter KF(N_STATES, N_MEASUREMENTS, N_INPUTS);

// handles callbacks
Point last_mouse_click = Point(-1, -1);
void mouseCallbackFunction(int event, int x, int y, int flags, void* userdata) {
    if (event == EVENT_LBUTTONDOWN) {
        cout << "reset KF" << endl;
        // TODO: RESET KF
        last_mouse_click.x = x;
        last_mouse_click.y = y;
        cout << "left_click " << x << "," << y << endl;
    }
}

// Detect an object
// returns (-1,-1) when no object found.
Point get_points_from_image(Mat *source) {
    return Point(-1, -1);

//    source->copyTo(res);
}

// Get an estimated point
Point get_estimated_point(Point raw) {
    return Point (-1, -1);
}

int main (int argc, char *arg[]) {
    // open and check the stream
    VideoCapture cap(0);
    if (cap.isOpened()) {
        cout << "Opening video " << CV_CAP_PROP_FRAME_WIDTH << "x" << CV_CAP_PROP_FRAME_HEIGHT << endl;
    } else {
        cout << "Could not open webcam" << endl;
        return -1;
    }
    
    // create a view to see the video.
    namedWindow("Webcam view", CV_WINDOW_AUTOSIZE);
    setMouseCallback("Webcam view", mouseCallbackFunction, NULL);

    while(1) {
        Mat frame;

        //break if a frame cannot be read
        if ( !cap.read(frame) ) {
            cout << "Cannot read a frame from the video stream" << endl;
            break;
        }

        // what do you want to track?
        if (last_mouse_click.x != -1) {
            // locate object
            Point a = get_points_from_image(&frame);
            Point predicted_point = get_estimated_point(a);
        }

        // display the final image
        imshow("Webcam view", frame);

        // break on user input
        if (waitKey(30) == 27) {
            cout << "ESC pressed by user" << endl;
            break;
        }
    }
    return 0;
}
