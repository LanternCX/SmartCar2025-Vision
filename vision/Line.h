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

enum ElementType {
    LINE,
    CURVE,
    CROSS,
    L_RING,
    R_RING
};


/**
 * @brief 边线数据
 */
typedef struct line_result {
    std::vector<cv::Point> line;
    std::vector<float> slope;
    ElementType type;
    cv::Size frame_size;
    int sample_dist = 1;
} line_result;

void filter_points(const std::vector<cv::Point>& pts_in, std::vector<cv::Point>& pts_out, int kernel);
void resample_points(const std::vector<cv::Point>& pts_in, std::vector<cv::Point>& pts_out, float dist);
void get_line_slope(const std::vector<cv::Point>& pts_in, std::vector<float>& angle_out, int angle_dist, int sample_dist);
void filter_line_slope(const std::vector<float>& angle_in, std::vector<float>& angle_out, int kernel);
void rebuild_line(const std::vector<float>& angle_in, std::vector<cv::Point>& pts_out, int dist, cv::Point origin);
int get_corner_count(const std::vector<int> &line, const int &threshold = 30);
bool is_line(const std::vector<cv::Point>& points, float threshold = 3.0);
std::vector<int> trans_line(const std::vector<cv::Point> &line, const cv::Size &size);
#endif