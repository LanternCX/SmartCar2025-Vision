#include <opencv2/opencv.hpp>

#include "Denoise.h"

/**
 * @file Denoise.cpp
 * @brief 图像降噪相关操作
 * @author Cao Xin
 * @date 2025-05-03
 */

/**
 * @brief 2x2 最小池化
 * @param src 输入图像
 * @param dst 输出图像
 * @return none
 * @author Cao Xin
 * @date 2025-05-03
 */
void minPool2x2(cv::Mat& src, cv::Mat& dst) {
    CV_Assert(src.type() == CV_8UC1);
    dst = cv::Mat::zeros(src.size(), CV_8UC1);

    for (int y = 0; y < src.rows; ++y) {
        for (int x = 0; x < src.cols; ++x) {
            int minVal = 255;
            for (int dy = 0; dy < 2; ++dy) {
                for (int dx = 0; dx < 2; ++dx) {
                    int ny = y + dy;
                    int nx = x + dx;
                    if (ny < src.rows && nx < src.cols) {
                        minVal = std::min(minVal, static_cast<int>(src.at<uchar>(ny, nx)));
                    }
                }
            }
            dst.at<uchar>(y, x) = static_cast<uchar>(minVal);
        }
    }
}

/**
 * @brief 3*3 高斯模糊
 * @param dst 输出图像
 * @return none
 * @author Cao Xin
 * @date 2025-05-03
 */
void gaussianBlur3x3(cv::Mat& src, cv::Mat& dst) {
    CV_Assert(src.type() == CV_8UC1);
    dst = cv::Mat::zeros(src.size(), CV_8UC1);

    const int kernel[3][3] = {
        {1, 2, 1},
        {2, 4, 2},
        {1, 2, 1}
    };

    for (int y = 1; y < src.rows - 1; ++y) {
        for (int x = 1; x < src.cols - 1; ++x) {
            int sum = 0;
            int weightSum = 0;
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    int val = src.at<uchar>(y + dy, x + dx);
                    int w = kernel[dy + 1][dx + 1];
                    sum += val * w;
                    weightSum += w;
                }
            }
            dst.at<uchar>(y, x) = static_cast<uchar>(sum / weightSum);
        }
    }
}

/**
 * @brief 3*3 腐蚀
 * @param src 输入图像
 * @param dst 输出图像
 * @return none
 * @author Cao Xin
 * @date 2025-05-03
 */
void erode3x3(const cv::Mat& src, cv::Mat& dst) {
    CV_Assert(src.type() == CV_8UC1);
    dst = cv::Mat::zeros(src.size(), CV_8UC1);

    // 处理内层的每个像素
    for (int y = 1; y < src.rows - 1; ++y) {
        for (int x = 1; x < src.cols - 1; ++x) {
            uchar minVal = 255;

            // 在3x3窗口内寻找最小值
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    minVal = std::min(minVal, src.at<uchar>(y + dy, x + dx));
                }
            }

            dst.at<uchar>(y, x) = minVal;
        }
    }
}

/**
 * @brief 3*3 膨胀
 * @param src 输入图像
 * @param dst 输出图像
 * @return none
 * @author Cao Xin
 * @date 2025-05-03
 */
void dilate3x3(const cv::Mat& src, cv::Mat& dst) {
    CV_Assert(src.type() == CV_8UC1);
    dst = cv::Mat::zeros(src.size(), CV_8UC1);

    // 处理内层的每个像素
    for (int y = 1; y < src.rows - 1; ++y) {
        for (int x = 1; x < src.cols - 1; ++x) {
            uchar maxVal = 0;

            // 在3x3窗口内寻找最大值
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    maxVal = std::max(maxVal, src.at<uchar>(y + dy, x + dx));
                }
            }

            dst.at<uchar>(y, x) = maxVal;
        }
    }
}

/**
 * @brief 图像降噪
 * @param image 输入图像
 * @return none
 * @author Cao Xin
 * @date 2025-05-03
 */
void denoise(cv::Mat& image) {
    CV_Assert(!image.empty() && image.type() == CV_8UC1);

    // 高斯模糊
    // gaussianBlur3x3(image, image);
    cv::GaussianBlur(image, image, cv::Size(3, 3), 0);

    // 腐蚀
    // erode3x3(image, image);
    cv::erode(image, image, cv::Mat());

    // 膨胀
    // dilate3x3(image, image);
    cv::dilate(image, image, cv::Mat());


    // 最小池化
    // minPool2x2(dilated, pooled);
}