#ifndef TRACK_H
#define TRACK_H


/**
 * @file Track.cpp
 * @brief 扫线相关头文件
 * @author Cao Xin
 * @date 2025-04-03
 */

#include <opencv2/opencv.hpp>
#include <vector>

/**
 * @brief 左右边线和中线
 */
typedef struct{
    // 左边线
    std::vector<cv::Point> left;
    // 右边线
    std::vector<cv::Point> right;
    // 中线
    std::vector<cv::Point> center;
}line_result;

/**
 * @brief 左右边线斜率
 */
typedef struct{
    // 左边线
    std::vector<float> left;
    // 右边线
    std::vector<float> right;
}line_slope_result;

line_result find_lines(cv::Mat img, cv::Point start, int block_size = 7, int max_points = 1000);
#endif