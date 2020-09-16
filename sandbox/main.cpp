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
#define BLUE {255,0,0}
#define SIZE 2

int main(int argc, char **argv) {
    string file = "rockland.mp4";
    if(argc == 2) {
        file = argv[1];
    }
    VideoCapture cap(file);
    if(!cap.isOpened()) {
        cout << "Error opening video" << endl;
        return -1;
    }
    featureExtractor feature_extractor;
    //VideoWriter out("new_extracted.mp4",CV_FOURCC('M','J','P','G'),10, Size(cap.get(CV_CAP_PROP_FRAME_WIDTH), cap.get(CV_CAP_PROP_FRAME_HEIGHT)));
    //if(!out.isOpened()) {cout << "Video Writer not opened" <<endl; return-1;}
    while (true) {
        Mat frame;
        cap >> frame;
        if(frame.empty()) break;
        auto ret = feature_extractor.extractFeatures(frame);
        for(auto const& i : ret.matchedKeypoints) {
            if(!i.empty()) {
                circle(frame,i.at(0).pt, SIZE, GREEN);
                line(frame, i.at(0).pt, i.at(1).pt, BLUE);
            }
        }
        Mat flipped_frame;
        flip(frame, flipped_frame, 1);
        //out.write(flipped_frame);
        //cout << cap.get(CV_CAP_PROP_FPS) << endl;
        cout << ret.retention_ratio << endl;
        imshow("frame", flipped_frame);
        char c = (char) waitKey(25);
        if(c==27) break;
    }
    cap.release();
    destroyAllWindows();
}