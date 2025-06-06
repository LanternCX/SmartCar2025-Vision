#include <cstdio>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <opencv2/videoio.hpp>

#include "Vision.h"
#include "Type.h"
#include "Perspective.h"

/**
 * @file Main.cpp
 * @brief 主函数文件
 * @author Cao Xin
 * @date 2025-04-03
 */

int run() {
    cv::VideoCapture cap(2);
    // cv::VideoCapture cap("../vedio/test.mp4");
    if (!cap.isOpened()) {
        std::cerr << "无法打开视频文件！" << std::endl;
        return -1;
    }

    cv::Mat frame;
    double fps = cap.get(cv::CAP_PROP_FPS);
    // double fps = 30;
    fps = 15;
    int delay = (fps > 0) ? static_cast<int>(1000 / fps) : 33;

    init_perspective();
    init_state();

    while (true) {
        cap >> frame;
        if (frame.empty()) {
            std::cerr << "无法读取视频帧或视频已结束！" << std::endl;
            break;
        }

        // 不知道为什么 resize 会导致透视变换不可用
        // cv::resize(frame, frame, cv::Size(), 0.5, 0.5);

        double alpha = 1.0;
        int beta = 50;
        frame.convertTo(frame, -1, alpha, beta);

        cv::imshow("raw", frame);
        process_img(frame);

        // 按 'q' 退出，delay 控制播放速度
        if (cv::waitKey(delay) == 'q') {
            break;
        }
    }

    // 释放视频资源
    cap.release();
    // 关闭窗口
    cv::destroyAllWindows();
    return 0;
}
int test() {
    mark_square();
    return 0;
}
int main() {
    return run();
}