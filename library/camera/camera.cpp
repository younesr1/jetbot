#include "camera.h"
#include "config.h"

// TODO: Use 60 FPS
camera::camera() : _pipeline("nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)" + std::to_string(CONFIG::CAMERA::WIDTH) 
                             + ", height=(int)" + std::to_string(CONFIG::CAMERA::HEIGHT) + ", format=(string)NV12, framerate=(fraction)"
                             + std::to_string(CONFIG::CAMERA::FPS) + "/1 ! nvvidconv flip-method=" + std::to_string(CONFIG::CAMERA::FLIP) 
                             + " ! video/x-raw, width=(int)" + std::to_string(CONFIG::CAMERA::WIDTH) + ", height=(int)" + std::to_string(CONFIG::CAMERA::HEIGHT)
                             + ", format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink"),
                    _cap(_pipeline, cv::CAP_GSTREAMER),
                    _img(),
                    _video("output.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), CONFIG::CAMERA::FPS, cv::Size(CONFIG::CAMERA::WIDTH, CONFIG::CAMERA::HEIGHT))
{
    if(!_cap.isOpened()) {
        std::cout << "Failed to open camera" << std::endl;
        exit(1);
    }
}

camera::~camera() {
    _cap.release();
    _video.release();
}

void camera::record() {
    std::chrono::steady_clock::time_point begin(std::chrono::steady_clock::now()), end(std::chrono::steady_clock::now());
    while (std::chrono::duration_cast<std::chrono::seconds>(end - begin) < CONFIG::CAMERA::RECORDTIME) {
        end = std::chrono::steady_clock::now();
        if (!_cap.read(_img)) {
		    std::cout<<"Capture read error"<<std::endl;
		    break;
	    }
        _video.write(_img);
    }
    std::cout << "Ending camera recording." << std::endl;
}