#include <opencv2/core/hal/interface.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

#include "Line.h"
#include "Math.h"
#include "Vision.h"
#include "Cross.h"

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
    track_result track = find_lines(gray, start);

    cv::Mat black = cv::Mat::zeros(frame.size(), CV_8UC1);
    cv::Size size(black.rows, black.cols);
    
    // 三角滤波
    filter_points(track.left.line, track.left.line, 15);
    filter_points(track.right.line, track.right.line, 15);

    track.left.line = get_perspective_line(track.left.line, size);
    track.right.line = get_perspective_line(track.right.line, size);
    
    // 等距采样
    std::vector<cv::Point> temp;
    resample_points(track.left.line, temp, track.left.sample_dist);
    track.left.line = temp;
    resample_points(track.right.line, temp, track.right.sample_dist);
    track.right.line = temp;

    // // 获取点集局部角度变化率
    // line_slope_result angle;
    // get_line_slope(track.left.line, track.left.slope, track.left.angle_dist, track.left.sample_dist);
    // get_line_slope(track.right.line, track.right.slope, track.right.angle_dist, track.right.sample_dist);

    // // 局部角度变化率非极大值抑制
    // filter_line_slope(angle.left, angle.left, 5);
    // filter_line_slope(angle.right, angle.right, 5);
    
    // track.left.corner = find_corners(track.left.slope, track.right.angle_dist, track.right.sample_dist);
    // track.right.corner = find_corners(track.right.slope, track.left.angle_dist, track.left.sample_dist);

    // corss_repair(track);
    std::vector<cv::Point> corner = find_corners(track.left.line);

    if(VISION_DEBUG){
        cv::Mat black = cv::Mat::zeros(frame.size(), CV_8UC1);
        
        for (auto pts : track.left.line) {
            cv::circle(black, pts, 1, 255);
        }
        for (auto pts : track.right.line) {
            cv::circle(black, pts, 1, 255);
        }

        for (auto pts : corner) {
            cv::circle(black, pts, 4, 255);
        }
        cv::imshow("Debug", gray);
        cv::imshow("IMG", frame);
        cv::imshow("black", black);
    }
    return {1, LINE};
}

