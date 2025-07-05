#include <cstdio>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <opencv2/videoio.hpp>

#include "Color.hpp"
#include "image.hpp"
#include "image_cv.hpp"
#include "Debug.h"

/**
 * @file Main.cpp
 * @brief 主函数文件
 * @author Cao Xin
 * @date 2025-04-03
 */
int run() {
    cv::Mat frame;

    cv::VideoCapture cap = getCap();
    image_cv_Init();

    Data_Settings();
    while (true) {
        cap >> frame;
        image_cv_zip(frame);
        cv::imshow("res", frame);
    }
    return 0;
}

int test() {
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

    while (true) {
        cap >> frame;
        if (frame.empty()) {
            std::cerr << "无法读取视频帧或视频已结束！" << std::endl;
            break;
        }
        
        // gray frame process
        image_cv_zip(frame);
        imageprocess();

        // rgb frame process
        cv::resize(frame, frame, cv::Size(80, 60));
        ring_judge(frame);

        // cv::Mat color_image(60, 80, CV_8UC3); // 彩色图像，60行80列，3通道

        // for (int i = 0; i < 60; ++i) {
        //     for (int j = 0; j < 80; ++j) {
        //         uchar v = img3[i][j];
        //         cv::Vec3b color;

        //         switch (v) {
        //             case 0:  color = cv::Vec3b(0, 0, 0);       break; // 黑色
        //             case 1:  color = cv::Vec3b(255, 255, 255); break; // 白色
        //             case 6:  color = cv::Vec3b(0, 0, 255);     break; // 红色 (BGR)
        //             case 7:  color = cv::Vec3b(0, 255, 0);     break; // 绿色
        //             case 8:  color = cv::Vec3b(255, 0, 0);     break; // 蓝色
        //             case 9:  color = cv::Vec3b(255, 255, 0);     break; // 蓝色
        //             default: color = cv::Vec3b(128, 128, 128); break; // 其它值设为灰色
        //         }

        //         color_image.at<cv::Vec3b>(i, j) = color;
        //     }
        // }
        // cv::resize(color_image, color_image, cv::Size(), 2.0, 2.0, cv::INTER_NEAREST);
        // draw_rgb_img(color_image);
        // debug(center);   

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

int main() {
    return test();
}