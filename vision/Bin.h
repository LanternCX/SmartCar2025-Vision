#ifndef BIN_H
#define BIN_H
#include <opencv2/opencv.hpp>

/**
 * 大津法二值化头文件
 */

void otsu_binarize(const cv::Mat& src, cv::Mat& dst, cv::Point topLeft = cv::Point(-1, -1), cv::Point bottomRight = cv::Point(-1, -1)) ;
uint8_t otsu_threshold(const cv::Mat& src, cv::Point topLeft = cv::Point(-1, -1), cv::Point bottomRight = cv::Point(-1, -1)) ;
#endif