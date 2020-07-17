#include <iostream>
#include <vector>
#include "opencv2/opencv.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/imgproc.hpp"
#include <vector>
#include "featureExtractor.h"
using namespace  cv;
using namespace std;

#define GREEN {0,255,0}
#define SIZE 2

#define MAX_FEATURES 1000
#define QUALITY 0.01
#define MINDIST 10


int main(int argc, char **argv) {
    string file = "output.avi";
    if(argc == 2) {
        file = argv[1];
    }
    VideoCapture cap(file);
    if(!cap.isOpened()) {
        cout << "Error opening video" << endl;
        return -1;
    }
    featureExtractor feature_extractor;
    while (true) {
        Mat frame;
        cap >> frame;
        if(frame.empty()) break;
        auto ret = feature_extractor.extractFeatures(frame);
        for(KeyPoint const& i : ret.locators) {
            circle(frame, i.pt, SIZE, GREEN);
        }
        imshow("frame", frame);
        char c = (char) waitKey(25);
        if(c==27) break;
    }
    cap.release();
    destroyAllWindows();
}