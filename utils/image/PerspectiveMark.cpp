#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

#include "Perspective.h"

std::vector<cv::Point2f> srcPoints;
cv::Mat frame, frameCopy;

void on_mouse(int event, int x, int y, int, void*) {
    if (event == cv::EVENT_LBUTTONDOWN && srcPoints.size() < 4) {
        cv::Point2f pt(x, y);
        std::cout << x << ' ' << y << '\n';
        srcPoints.push_back(pt);
        circle(frameCopy, pt, 1, cv::Scalar(0, 0, 255), cv::FILLED);
        imshow("Camera Frame - Select 4 Points", frameCopy);
    }
}

int mark_square() {
    cv::VideoCapture cap(0);  // 改成你摄像头编号
    if (!cap.isOpened()) {
        std::cerr << "无法打开摄像头！" << std::endl;
        return -1;
    }

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

    std::cout << srcPoints.data() << '\n';

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