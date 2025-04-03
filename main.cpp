#include <cstdio>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>

#include "Utils.h"

int main() {
    // 打开视频文件（替换 "video.mp4" 为你的视频文件路径）
    cv::VideoCapture cap("../vedio/test.mp4");  // 修改这一行
    if (!cap.isOpened()) {
        std::cerr << "无法打开视频文件！" << std::endl;
        return -1;
    }

    cv::Mat frame;
    // 获取视频的帧率（可选，用于更精确的控制）
    double fps = cap.get(cv::CAP_PROP_FPS);
    // fps = 15;
    int delay = (fps > 0) ? static_cast<int>(1000 / fps) : 33;

    while (true) {
        cap >> frame;
        if (frame.empty()) {
            std::cerr << "无法读取视频帧或视频已结束！" << std::endl;
            break;
        }
        
        cv::resize(frame, frame, cv::Size(), 0.5, 0.5);

        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        
        cv::Mat bin;
        binarizeWithOtsu(gray, bin);

        cv::imshow("color", frame);
        cv::imshow("gray", gray);
        cv::imshow("bin", bin);

        lineDetection(bin, frame);

        cv::imshow("res", frame);

        // 按 'q' 退出，delay 控制播放速度
        if (cv::waitKey(delay) == 'q') {
            break;
        }
    }
    
    cap.release(); // 释放视频资源
    cv::destroyAllWindows(); // 关闭窗口
    return 0;
}