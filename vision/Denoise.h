#ifndef _DENOISE_H_
#define _DENOISE_H_

/**
 * @file Denoise.h
 * @brief 图像降噪相关操作头文件
 * @author Cao Xin
 * @date 2025-05-03
 */

#include <opencv2/opencv.hpp>
void denoise(cv::Mat& image, const cv::Point center, int size);
#endif