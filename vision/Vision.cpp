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
#include "Perspective.h"
#include "Track.h"
#include "Vision.h"
#include "Cross.h"
#include "Debug.h"

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

    std::vector<cv::Point> temp;
    // 三角滤波
    filter_points(track.left.line, temp, 15);
    track.left.line = temp;
    filter_points(track.right.line, temp, 15);
    track.right.line = temp;

    track.left.line = get_perspective_line(track.left.line, size);
    track.right.line = get_perspective_line(track.right.line, size);
    
    // 等距采样
    resample_points(track.left.line, temp, track.left.sample_dist);
    track.left.line = temp;
    resample_points(track.right.line, temp, track.right.sample_dist);
    track.right.line = temp;

    // 采样之后好像有一些点的值比较异常所以筛一遍
    temp.clear();
    for (cv::Point pts : track.left.line) {
        if (pts.x <= 3 || pts.y <= 3 || pts.x >= width - 3 || pts.y >= width - 3) {
            continue;
        }
        temp.push_back(pts);
    }
    track.left.line = temp;

    temp.clear();
    for (cv::Point pts : track.right.line) {
        if (pts.x <= 3 || pts.y <= 3 || pts.x >= width - 3 || pts.y >= width - 3) {
            continue;
        }
        temp.push_back(pts);
    }
    track.right.line = temp;

    // 三角滤波
    filter_points(track.left.line, temp, 15);
    track.left.line = temp;
    filter_points(track.right.line, temp, 15);
    track.right.line = temp;

    auto type = get_element_type(track);
    debug(type);

    if(VISION_DEBUG){
        cv::Mat black = cv::Mat::zeros(frame.size(), CV_8UC1);
        for (auto pts : track.left.line) {
            cv::circle(black, pts, 1, 255);
        }
        for (auto pts : track.right.line) {
            cv::circle(black, pts, 1, 255);
        }
        cv::imshow("black", black);
    }
    return {1, LINE};
}

