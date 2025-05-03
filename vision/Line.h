#ifndef _LINE_H_
#define _LINE_H_

/**
 * @file Line.h
 * @brief 边线处理相关操作头文件
 * @author Cao Xin
 * @date 2025-05-03
 */

#include <opencv2/opencv.hpp>
void blur_points(const std::vector<cv::Point>& pts_in, std::vector<cv::Point>& pts_out, int kernel);
void resample_points(const std::vector<cv::Point>& pts_in, std::vector<cv::Point>& pts_out, float dist);
void get_line_slope(const std::vector<cv::Point>& pts_in, std::vector<float>& angle_out, int dist);
void filter_line_slope(const std::vector<float>& angle_in, std::vector<float>& angle_out, int kernel);
#endif