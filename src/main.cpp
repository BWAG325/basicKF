//
// Created by mmz on 2025/8/24.
//
#include <iostream>
#include <opencv2/opencv.hpp>

#include "tools/types.h"
#include "cam.h"
#include "kalman/caKalman.h"

int main() {
    Cam usbCam(2);
    usbCam.startThread();

    using clock = std::chrono::steady_clock;
    auto last_time = clock::now();
    int frame_count = 0;
    double fps = 0.0;

    cv::Mat frame;
    while (true) {
        const auto key = cv::waitKey(1);
        if (usbCam.getFrame(frame)) {


            //NOTE 帧率显示部分 程序帧率
            ++frame_count;
            // 计算时间差（秒）
            auto now = clock::now();
            double elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(now - last_time).count();
            // 每秒更新一次 FPS（避免频繁计算）
            if (elapsed >= 0.05) {
                fps = frame_count / elapsed;
                frame_count = 0;
                last_time = now;
            }
            // 显示 FPS 到图像
            cv::putText(frame, "FPS: " + std::to_string(static_cast<int>(std::round(fps))), cv::Point(20, 40),
                        cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);
            cv::imshow("frame", frame);
        }
        //NOTE ESC
        if (key == 27) {
            break;
        }
    }
    cv::destroyAllWindows();

    return 0;
}