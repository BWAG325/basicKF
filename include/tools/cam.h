//
// Created by mmz on 25-7-12.
//

#ifndef CAM_H
#define CAM_H

#include <opencv2/opencv.hpp>
#include <thread>
#include <mutex>

class Cam {
    bool seeTimeCost;

    double frameRate;
    double w;
    double h;
    double exposure;

    cv::Mat frame;
    cv::VideoCapture cap;
    int camID;
    bool emptyFrame;

    bool stopFlag;
    std::mutex mtx;
    std::thread thread;

    void camSet();
    void capture();

    int frameCount;
    double totalTime;
    double minTime;
    double maxTime;

public:
    explicit Cam(int id);
    ~Cam();

    void startThread();

    bool getFrame(cv::Mat& mat);
};


#endif //CAM_H
