#ifndef _PERSPECTIVE_H_
#define _PERSPECTIVE_H_

/**
 * @file Perspective.h
 * @brief 图像逆透视相关操作头文件
 * @author Cao Xin
 * @date 2025-04-13
 */

#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>

const int FRAME_WIDTH = 300;
const int FRAME_HEIGHT = 300;
void init_perspective();
cv::Mat get_perspective_img(cv::Mat image);
std::vector<int> get_perspective_line(const std::vector<int>& line, cv::Size size);
cv::Point get_perspective_pt(cv::Point src);
// void mark_square();
#endif