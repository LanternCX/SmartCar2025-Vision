#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>

/**
 * @brief 使用大津法计算灰度图像的动态阈值
 * @param src 输入的灰度图像 (cv::Mat, 单通道)
 * @return 计算得到的阈值 (uint8_t)
 */
 uint8_t otsuThreshold(const cv::Mat& src) {
    // 检查输入图像是否为灰度图像
    if (src.channels() != 1) {
        throw std::runtime_error("Input image must be grayscale (single channel).");
    }

    // 定义灰度级数 (0-255)
    const int grayScale = 256;
    // 初始化灰度直方图数组
    int hist[grayScale] = {0};

    // 计算灰度直方图
    for (int i = 0; i < src.rows; i++) {
        // 获取当前行的指针
        const uchar* ptr = src.ptr<uchar>(i);
        for (int j = 0; j < src.cols; j++) {
            // 统计每个灰度值的像素数
            hist[ptr[j]]++;
        }
    }

    // 计算总像素数
    int totalPixels = src.rows * src.cols;
    // 初始化灰度积分
    double sum = 0;
    for (int i = 0; i < grayScale; i++) {
        // 计算所有像素的灰度加权和
        sum += i * hist[i];
    }

    // 初始化背景灰度积分
    double sumB = 0;
    // 初始化背景像素数
    int wB = 0;
    // 初始化前景像素数
    int wF = 0;
    // 初始化最大类间方差
    double maxVariance = 0;
    // 初始化最佳阈值
    uint8_t threshold = 0;

    // 大津法计算最佳阈值
    for (int t = 0; t < grayScale; t++) {
        // 背景像素数累加
        wB += hist[t];
        // 避免除以零
        if (wB == 0) continue;

        // 计算前景像素数
        wF = totalPixels - wB;
        // 所有像素已处理完则退出
        if (wF == 0) break;

        // 背景灰度积分累加
        sumB += t * hist[t];

        // 计算背景均值
        double mB = sumB / wB;
        // 计算前景均值
        double mF = (sum - sumB) / wF;

        // 计算类间方差
        double variance = (double)wB * (double)wF * (mB - mF) * (mB - mF);

        // 更新最大方差和阈值
        if (variance > maxVariance) {
            maxVariance = variance;
            threshold = static_cast<uint8_t>(t);
        }
    }

    // 返回计算得到的阈值
    return threshold;
}

/**
 * @brief 使用大津法对灰度图像进行二值化
 * @param src 输入的灰度图像 (cv::Mat, 单通道)
 * @param dst 输出的二值化图像 (cv::Mat, 单通道)
 */
void binarizeWithOtsu(const cv::Mat& src, cv::Mat& dst) {
    // 检查输入图像是否为空
    if (src.empty()) {
        throw std::runtime_error("Input image is empty.");
    }
    // 检查输入图像是否为灰度图像
    if (src.channels() != 1) {
        throw std::runtime_error("Input image must be grayscale (single channel).");
    }

    // 计算大津法阈值
    uint8_t threshold = otsuThreshold(src);

    // 创建输出图像，初始化为全黑
    dst = cv::Mat::zeros(src.size(), CV_8UC1);

    // 二值化处理
    for (int i = 0; i < src.rows; i++) {
        // 获取输入图像当前行的指针
        const uchar* srcPtr = src.ptr<uchar>(i);
        // 获取输出图像当前行的指针
        uchar* dstPtr = dst.ptr<uchar>(i);
        for (int j = 0; j < src.cols; j++) {
            // 根据阈值进行二值化，大于阈值为255，否则为0
            dstPtr[j] = (srcPtr[j] > threshold) ? 255 : 0;
        }
    }
}

int main() {
    // 打开视频文件（替换 "video.mp4" 为你的视频文件路径）
    cv::VideoCapture cap("../vedio/test.mp4");  // 修改这一行
    if (!cap.isOpened()) {
        std::cerr << "无法打开视频文件！" << std::endl;
        return -1;
    }

    cv::Mat frame;
    // 获取视频的帧率（可选，用于更精确的控制）
    double fps = cap.get(cv::CAP_PROP_FPS);
    int delay = (fps > 0) ? static_cast<int>(1000 / fps) : 33; // 计算每帧延迟时间（毫秒）

    while (true) {
        cap >> frame; // 读取视频帧
        if (frame.empty()) {
            std::cerr << "无法读取视频帧或视频已结束！" << std::endl;
            break;
        }
        
        cv::resize(frame, frame, cv::Size(), 0.5, 0.5);

        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        
        cv::Mat bin;
        binarizeWithOtsu(gray, bin);

        cv::imshow("color", frame);
        cv::imshow("gray", gray);
        cv::imshow("bin", bin);

        // 按 'q' 退出，delay 控制播放速度
        if (cv::waitKey(delay) == 'q') {
            break;
        }
    }

    cap.release(); // 释放视频资源
    cv::destroyAllWindows(); // 关闭窗口
    return 0;
}