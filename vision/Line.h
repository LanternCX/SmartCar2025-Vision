#ifndef _LINE_H_
#define _LINE_H_

/**
 * @file Line.h
 * @brief 边线处理相关操作头文件
 * @author Cao Xin
 * @date 2025-05-03
 */

#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

/**
 * @brief 左右边线斜率
 */
typedef struct {
    // 左边线
    std::vector<float> left;
    // 右边线
    std::vector<float> right;
} line_slope_result;

enum ElementType {
    LINE,
    CURVE,
    CROSS,
    
};


/**
 * @brief 边线数据
 */
typedef struct line_result{
    std::vector<cv::Point> line;
    std::vector<float> slope;
    ElementType type;
    int angle_dist = 3;
    int sample_dist = 3;
} line_result;

void filter_points(const std::vector<cv::Point>& pts_in, std::vector<cv::Point>& pts_out, int kernel);
void resample_points(const std::vector<cv::Point>& pts_in, std::vector<cv::Point>& pts_out, float dist);
void get_line_slope(const std::vector<cv::Point>& pts_in, std::vector<float>& angle_out, int angle_dist, int sample_dist);
void filter_line_slope(const std::vector<float>& angle_in, std::vector<float>& angle_out, int kernel);
void rebuild_line(const std::vector<float>& angle_in, std::vector<cv::Point>& pts_out, int dist, cv::Point origin);
std::vector<cv::Point> find_corners(const std::vector<cv::Point>& points, double angleThresholdDegrees = 90.0);
#endif