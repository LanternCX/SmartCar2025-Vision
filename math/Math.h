#ifndef _MATH_H_
#define _MATH_H_

#include <vector>
#include <opencv2/opencv.hpp>

struct line_params {
    // 斜率
    float slope;
    // 截距
    float intercept; 
    // 垂直线 x = c
    float c;
    // 是否为垂直线
    bool is_vertical;
};

float minmax(float a, float min, float max);
float min(float a, float b);
float max(float a, float b);
line_params fit_line(const std::vector<cv::Point2f>& points);
#endif