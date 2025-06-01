/**
 * @file Track.cpp
 * @brief 扫线相关头文件
 * @author Cao Xin
 * @date 2025-04-03
 */

#pragma once

#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include "Line.h"
/**
 * @brief 左右边线和中线
 */
typedef struct {
    // 左边线
    line_result left;
    
    // 右边线
    line_result right;
    
    // 元素类型
    ElementType type;

    // 最终拟合的中线
    std::vector<cv::Point> center;
} track_result;

track_result find_lines(cv::Mat img, cv::Point start, int block_size = 7, int max_points = 1000);
ElementType get_element_type(track_result &track);