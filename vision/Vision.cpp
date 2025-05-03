#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

#include "Math.h"
#include "Vision.h"

/**
 * @file Vision.cpp
 * @brief 图像处理主要操作
 * @author Cao Xin
 * @date 2025-04-13
 */

/**
 * @brief 绘制边线
 * @param line 边线
 * @param image 输入的图像
 * @return none
 * @author Cao Xin
 * @date 2025-04-03
 */
void draw_line(std::vector<int> line, cv::Mat& image){
    for(size_t y = 0; y < line.size(); y++){
        if (image.type() == CV_8UC3) {
            cv::circle(image, cv::Point(line[y], y), 2, cv::Vec3b(255, 255, 255), -1);
        } else {
            cv::circle(image, cv::Point(line[y], y), 2, 255, -1);
        }
    }
}

/**
 * @brief 图像处理主函数
 * @param frame 输入的图像
 * @return 中线识别的结果
 * @author Cao Xin
 * @date 2025-04-13
 */
vision_result process_img(cv::Mat frame){
    // 彩色转灰度
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    // 图像降噪
    // denoise(gray);

    // 获取长宽
    int height = frame.cols;
    int width = frame.rows;
    if(width < height){
        std::swap(height, width);
    }

    // 扫线
    cv::Point start = {width / 2, height - 10};
    line_result line = find_lines(gray, start);

    // 三角滤波
    blur_points(line.left, line.left, 15);
    blur_points(line.right, line.right, 15);

    // 等距采样
    std::vector<cv::Point> temp;
    resample_points(line.left, temp, 3.0f);
    line.left = temp;
    resample_points(line.right, temp, 3.0f);
    line.right = temp;

    // 获取点集局部角度变化率
    line_slope_result angle;
    get_line_slope(line.left, angle.left, (5.0f / 3.0f));
    get_line_slope(line.right, angle.right, (5.0f / 3.0f));

    // 局部角度变化率非极大值抑制
    filter_line_slope(angle.left, angle.left, 5);
    filter_line_slope(angle.right, angle.right, 5);

    if(VISION_DEBUG){
        for(cv::Point pts : line.left){
            cv::circle(frame, pts, 2, cv::Vec3b(255, 0, 0), -1);
        }
        for(cv::Point pts : line.right){
            cv::circle(frame, pts, 2, cv::Vec3b(0, 255, 0), -1);
        }
        cv::imshow("Debug", frame);
    }
    return {1, LINE};
}

