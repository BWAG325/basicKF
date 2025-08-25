//
// Created by mmz on 25-7-12.
//

#include "tools/cam.h"

Cam::Cam(const int id) : camID(id), emptyFrame(true), stopFlag(false) {
    seeTimeCost = false;
    //TODO 增加参数设置接口，并支持启动过程中热切换
    frameRate = 200;
    w = 1280;
    h = 720;
    exposure = 150;

    frameCount = 0;
    totalTime = 0.0;
    minTime = 10000.0;
    maxTime = 0.0;
    camSet();
}

Cam::~Cam() {
    stopFlag = true;
    if (thread.joinable()) {
        thread.detach();
    }
    cap.release();
}

void Cam::camSet() {
    cap.open(camID, cv::CAP_V4L2);
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    // cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('Y','U','Y','V'));
    // bool success = cap.set(cv::CAP_PROP_HW_ACCELERATION, cv::VIDEO_ACCELERATION_ANY);
    // if (!success) {
    //     std::cerr << "硬件加速设置失败,后端可能不支持" << std::endl;
    // }
    cap.set(cv::CAP_PROP_FRAME_WIDTH, w);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, h);
    cap.set(cv::CAP_PROP_FPS, frameRate);
    // cap.set(cv::CAP_PROP_AUTO_WB, 0); //设置为0表示关闭自动白平衡
    // cap.set(cv::CAP_PROP_WHITE_BALANCE_BLUE_U, 0); //关闭蓝色通道自动调整
    // cap.set(cv::CAP_PROP_WHITE_BALANCE_RED_V, 0); //关闭红色通道自动调整
    cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 1);
    cap.set(cv::CAP_PROP_EXPOSURE, exposure);

    if (!cap.isOpened()) {
        emptyFrame = true;
        std::cout << "Can't open this cam" << std::endl;
    }
}

void Cam::capture() {
    using namespace std::chrono_literals;

    while (!stopFlag) {
        cv::Mat img;
        if (cap.grab()) {
            auto start = std::chrono::high_resolution_clock::now();
            cap.retrieve(img);
            auto end = std::chrono::high_resolution_clock::now();
            double elapsed_ms = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0;
            mtx.lock();
            frame = img.clone();
            mtx.unlock();
            emptyFrame = false;
            // 更新统计数据
            frameCount++;
            totalTime += elapsed_ms;
            minTime = std::min(minTime, elapsed_ms);
            maxTime = std::max(maxTime, elapsed_ms);
            double avgTime = totalTime / frameCount;

            if (seeTimeCost) {
                // 在图像上绘制时间信息
                std::string timeStr = "Cur: " + std::to_string(elapsed_ms).substr(0, 5) + " ms";
                std::string avgStr = "Avg: " + std::to_string(avgTime).substr(0, 5) + " ms";
                std::string minStr = "Min: " + std::to_string(minTime).substr(0, 5) + " ms";
                std::string maxStr = "Max: " + std::to_string(maxTime).substr(0, 5) + " ms";

                cv::putText(frame, timeStr, cv::Point(20, 80), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);
                cv::putText(frame, avgStr, cv::Point(20, 120), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 200, 255),
                            2);
                cv::putText(frame, minStr, cv::Point(20, 160), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 100, 0),
                            2);
                cv::putText(frame, maxStr, cv::Point(20, 200), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 150, 255),
                            2);
            }
        } else {
            emptyFrame = true;
            std::this_thread::sleep_for(1s);
            std::cout << "Can't grab frame,reset cam" << std::endl;
            camSet();
        }
    }
}

void Cam::startThread() {
    thread = std::thread(&Cam::capture, this);
}

bool Cam::getFrame(cv::Mat& mat) {
    if (emptyFrame) {
        return false;
    }
    mat = frame.clone();
    return true;
}


