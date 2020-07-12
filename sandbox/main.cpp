#include <iostream>
#include <vector>
#include "opencv2/opencv.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/imgproc.hpp"
#include <vector>
using namespace  cv;
using namespace std;

int main() {
    VideoCapture cap("output.avi");
    vector<KeyPoint> kp;
    Mat desc;
    Scalar sclr = {0,255,0};
    Ptr<ORB> orb = ORB::create();
    if(!cap.isOpened()) {
        cout << "Error opening video" << endl;
        return -1;
    }
    while (true)
    {
        Mat frame;
        cap >> frame;
        if(frame.empty()) break;
        orb->detectAndCompute(frame, Mat(), kp, desc);
        for(int i = 0; i < kp.size(); i++) {
            circle(frame, kp[i].pt, 2, sclr);
        }
        imshow("frame", frame);
        char c = (char) waitKey(25);
        if(c==27) break;
    }
    cap.release();
    destroyAllWindows();
}