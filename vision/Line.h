#ifndef LINE_H
#define LINR_H

/**
 * 扫线头文件
 */

#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

typedef struct{
    std::vector<cv::Point> left;
    std::vector<cv::Point> right;
    std::vector<cv::Point> center;
}line_result;

typedef struct{
    std::vector<float> left;
    std::vector<float> right;
}line_angle_result;

line_result find_lines(cv::Mat img, cv::Point start, int block_size = 7, int clip_value = 20, int max_points = 1000);
void blur_points(const std::vector<cv::Point>& pts_in, std::vector<cv::Point>& pts_out, int kernel);
void resample_points(const std::vector<cv::Point>& pts_in, std::vector<cv::Point>& pts_out, float dist);
void get_line_angle(const std::vector<cv::Point>& pts_in, std::vector<float>& angle_out, int dist);
void filter_line_angle(const std::vector<float>& angle_in, std::vector<float>& angle_out, int kernel);
#endif