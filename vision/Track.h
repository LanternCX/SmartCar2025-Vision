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
#include "Line.h"
/**
 * @brief 左右边线和中线
 */
typedef struct{
    // 左边线
    line_result left;
    // 右边线
    line_result right;
} track_result;

track_result find_lines(cv::Mat img, cv::Point start, int block_size = 7, int max_points = 1000);
#endif