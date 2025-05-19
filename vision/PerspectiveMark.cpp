#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

/**
 * @file PerspectiveMark.cpp
 * @brief 图像逆透视标定 (Desktop Only)
 * @author Cao Xin
 * @date 2025-04-13
 */

#include "Perspective.h"

std::vector<cv::Point2f> srcPoints;
cv::Mat frame, frameCopy;

/**
 * @brief 鼠标点击事件回调函数
 * @param event 事件编号
 * @param x 鼠标坐标 x
 * @param y 鼠标坐标 y
 * @return none
 * @author Cao Xin
 * @date 2025-04-13
 */
void on_mouse(int event, int x, int y, int, void*) {
    if (event == cv::EVENT_LBUTTONDOWN && srcPoints.size() < 4) {
        cv::Point2f pt(x, y);
        srcPoints.push_back(pt);
        circle(frameCopy, pt, 1, cv::Scalar(0, 0, 255), cv::FILLED);
        imshow("Camera Frame - Select 4 Points", frameCopy);
    }
}

/**
 * @brief 逆透视标定主函数
 * @return none
 * @author Cao Xin
 * @date 2025-04-13
 */
int mark_square() {
    // 输入源
    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        std::cerr << "无法打开摄像头！" << std::endl;
        return -1;
    }

    // 输入帧
    cap >> frame;
    if (frame.empty()) {
        std::cerr << "无法读取图像！" << std::endl;
        return -1;
    }


    frameCopy = frame.clone();
    namedWindow("Camera Frame - Select 4 Points", cv::WINDOW_AUTOSIZE);
    cv::setMouseCallback("Camera Frame - Select 4 Points", on_mouse);

    imshow("Camera Frame - Select 4 Points", frameCopy);

    while (srcPoints.size() < 4) {
        if (cv::waitKey(1) == 27) return 0;
    }
    // 左上, 右上, 右下, 左下
    std::vector<cv::Point2f> srcPoints = {
        cv::Point2f(220, 267),
        cv::Point2f(418, 267),
        cv::Point2f(502, 463),
        cv::Point2f(139, 461) 
    };

    // 左上, 右上, 右下, 左下
    std::vector<cv::Point2f> dstPoints = {
        cv::Point2f(0, 0),
        cv::Point2f(FRAME_WIDTH - 1, 0),
        cv::Point2f(FRAME_WIDTH - 1, FRAME_HEIGHT - 1),
        cv::Point2f(0, FRAME_HEIGHT - 1)
    };

    // 计算变换矩阵
    cv::Mat M = getPerspectiveTransform(srcPoints, dstPoints);

    while (true) {
        cap >> frame;
        if (frame.empty()) break;

        // 用计算出来的透视变换矩阵对整幅图像进行变换
        cv::Mat warped;
        // 输出图像尺寸与原图一致
        warpPerspective(frame, warped, M, cv::Size(FRAME_WIDTH, FRAME_HEIGHT));
        imshow("Transformed Whole View", warped);
        imshow("Original Frame", frame);

        char key = cv::waitKey(1);
        // ESC退出
        if (key == 27) break;
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}