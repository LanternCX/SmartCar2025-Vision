#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

#include "Math.h"

#include "Perspective.h"
#include "Vision.h"

void draw_line(std::vector<int> line, cv::Mat& image){
    for(int y = 0; y < line.size(); y++){
        if (image.type() == CV_8UC3) {
            cv::circle(image, cv::Point(line[y], y), 2, cv::Vec3b(255, 255, 255), -1);
        } else {
            cv::circle(image, cv::Point(line[y], y), 2, 255, -1);
        }
    }
}

bool is_line(const std::vector<int>& res, float threshold = 2.0) {
    // 提取有效点
    std::vector<cv::Point2f> points;
    for (size_t y = 0; y < res.size(); ++y) {
        if (res[y] != -1) {
            points.emplace_back(static_cast<float>(res[y]), static_cast<float>(y));
        }
    }

    // 如果点数太少，无法判断，默认为曲线
    if (points.size() < 3) {
        return false;
    }

    // 拟合直线
    line_params params = fit_line(points);

    // 计算每个点到直线的距离
    float total_distance = 0;
    if (!params.is_vertical) {
        // 非垂直线：y = mx + b
        for (const auto& p : points) {
            // 点到直线距离公式：|mx - y + b| / sqrt(m^2 + 1)
            float distance = std::abs(params.slope * p.x - p.y + params.intercept) /
                           std::sqrt(params.slope * params.slope + 1);
            total_distance += distance;
        }
    } else {
        // 垂直线：x = c
        for (const auto& p : points) {
            float distance = std::abs(p.x - params.c);
            total_distance += distance;
        }
    }

    // 计算平均偏差
    float avg_distance = total_distance / points.size();

    // 根据阈值判断
    return avg_distance < threshold;
}

vision_result process_img(cv::Mat frame){
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    
    cv::Mat bin;
    otsu_binarize(gray, bin);

    line_result line = line_detection(bin, frame);

    cv::Mat black = cv::Mat::zeros(frame.size(), CV_8UC1);
    cv::Size size(black.rows, black.cols);
    line.left = get_perspective_line(line.left, size);
    line.right = get_perspective_line(line.right, size);
    line.center = get_perspective_line(line.center, size);

    if(VISION_DEBUG){
        draw_line(line.left, black);
        draw_line(line.right, black);
        draw_line(line.center, black);
        cv::imshow("res", frame);
        cv::imshow("bin", bin);
        cv::imshow("per", black);
        cv::imshow("test", get_perspective_img(frame));
    }

    Type type = is_line(line.center) ? LINE : CURVE;
    return {line.center[line.center.size() - 10], type};
}

