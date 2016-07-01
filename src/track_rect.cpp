/*  track_rect.cpp
 *  author: Daniel D'Souza
 *  purpose: tracks a blue rectangle
 */

#include <chrono>
#include <deque>
#include <iostream>
#include <mutex>
#include <thread>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>

#define DRAW_CROSS( img, center, color, d )                                 \
line( img, Point( center.x - d, center.y - d ), Point( center.x + d, center.y + d ), color, 2, CV_AA, 0); \
line( img, Point( center.x + d, center.y - d ), Point( center.x - d, center.y + d ), color, 2, CV_AA, 0 )

using namespace cv;
using namespace std;

std::deque<cv::Point> detected_object, estimated_object;
// std::queue<cv::Point> estimated_object, measured_object;
std::mutex shared_mutex;

void ProcessImage() {
    Mat src, hsv, range;
    SimpleBlobDetector::Params params;
    
    
    params.filterByCircularity=1;
    params.minCircularity=0.01;
    params.filterByConvexity=1;
    params.minConvexity=0.01;

    cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);
    VideoCapture cap(0);

    // are images being read from the camera?
    if (!cap.isOpened()) {
        cout << "ERROR opening stream" << endl;
        return;
    }
    
    namedWindow("Feed", CV_WINDOW_AUTOSIZE);    

    double dWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH), dHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT);
    cout << "Frame size : " << dWidth << " x " << dHeight << endl;
    
    // loop while a image can be read 
    while(cap.read(src)) {
        // filter out noise and keep yellow
        medianBlur(src, src, 27);
        cvtColor(src, hsv, CV_BGR2HSV);
        inRange(hsv, 
                Scalar(90, 100, 100),
                Scalar(130, 255, 255),
                range);

        // blob detection
        vector<KeyPoint> keypoints;
        detector->detect(range, keypoints);
        drawKeypoints(range, keypoints, range, Scalar(0, 0, 255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        // push location
        {
            lock_guard<std::mutex> lock(shared_mutex);
            // add new point
            if (!keypoints.empty()) {
                detected_object.push_back(keypoints.front().pt);
                DRAW_CROSS(range, keypoints.front().pt, Scalar(0,0,255), 5);
            }
            if (!estimated_object.empty()) {
                // remove oldest points
                while (estimated_object.size() > 20)
                    estimated_object.pop_front();
                // draw most recent estimate
                DRAW_CROSS(range, estimated_object.back(), Scalar(0,255,0), 5);
            }
            // draw trails
            for (deque<Point>::iterator it = detected_object.begin(); it < detected_object.begin() + 19 && it != detected_object.end(); it++) {
                line(range, *it, *(it+1), Scalar(0,0,255), 1);
            }
            for (deque<Point>::iterator it = estimated_object.begin(); 
                    it != estimated_object.end(); 
                    it++) 
            {
                // line(range, *it, *(it+1), Scalar(0,255,0), 1); // this is broken, no clue why.
            }
        } //end critical section
        
        imshow("Feed", range);

        // kill window when ESC is pressed. but just use ctrl+c
        if (waitKey(30) == 27) {
            cout << "ESC" << endl;
            destroyWindow("Feed");
            break;
        }
 
    }
}

void TrackCenter() {
    //Setup Kalmin Filter
    KalmanFilter KF(4, 2, 0); //states, measurements, inputs

    // Setup matrices
    KF.transitionMatrix = Mat::eye(4, 4, CV_32F);               // relationship between time t and t-1 
    Mat_<float> measurement(2,1); measurement.setTo(Scalar(0));
    
    // Setup initial states. Tuning done here...
    for (int i=0; i<4; i++)
        KF.statePre.at<float>(i) = 0;                           // initial state values
    setIdentity(KF.measurementMatrix);
    setIdentity(KF.processNoiseCov, Scalar::all(1e-4));         // our system will have low noise and error
    setIdentity(KF.measurementNoiseCov, Scalar::all(10));
    setIdentity(KF.errorCovPost, Scalar::all(.1));

    while(1) {
        // Do prediction while we wait for a new result
        Mat prediction = KF.predict();
        Point predictPt(prediction.at<float>(0), prediction.at<float>(1));
            
        // Acquire lock - critical section
        {
            lock_guard<std::mutex> lock(shared_mutex);
            estimated_object.push_back(predictPt);
            if (detected_object.size() < 20)
                continue;
            else {
                while (detected_object.size() > 20)
                    detected_object.pop_front();
            }
            // update with data
            measurement(0) = detected_object.back().x;
            measurement(1) = detected_object.back().y;
            Mat estimated = KF.correct(measurement);
            //estimated_object.push_back(Point(estimated.at<float>(0), estimated.at<float>(1)));
        } // end critical section
    }        
}    

int main (int argc, char* const argv[]) {
    // Attempt to find object.
    thread process(ProcessImage);
    thread track(TrackCenter);
    process.join();
    // kil tracking thread
    
    return 0;
}

