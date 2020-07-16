#include <iostream>
#include <vector>
#include "opencv2/opencv.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/imgproc.hpp"
#include <vector>
using namespace  cv;
using namespace std;

#define GREEN {0,255,0}
#define SIZE 2

#define MAX_FEATURES 1000
#define QUALITY 0.01
#define MINDIST 10

void color2gray(const Mat& frame, Mat& bwframe);
void drawCircles(const vector<Point2f>& v, Mat& frame);
void findFeaturePoints(const Mat& f, vector<Point2f>& v);

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
    vector<Point2f> points;
    Mat frame;
    while (true)
    {
        cap >> frame;
        if(frame.empty()) break;
        findFeaturePoints(frame, points);
        drawCircles(points, frame);
        imshow("frame", frame);
        char c = (char) waitKey(25);
        if(c==27) break;
    }
    cap.release();
    destroyAllWindows();
}

void color2gray(const Mat& frame, Mat& bwframe) {
    cvtColor(frame, bwframe, COLOR_BGR2GRAY);
}

void drawCircles(const vector<Point2f>& v, Mat& frame) {
    for(Point2f const& i : v) {
        circle(frame, i, SIZE, GREEN);
    }
}

void findFeaturePoints(const Mat& f, vector<Point2f>& v) {
    Mat bwf;
    color2gray(f, bwf);
    goodFeaturesToTrack(bwf, v, MAX_FEATURES, QUALITY, MINDIST);
}