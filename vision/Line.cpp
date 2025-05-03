#include <algorithm>
#include <opencv2/core/hal/interface.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

#include "Vision.h"

/**
 * @brief 给图像绘制黑色边框
 * @param image 输入的图像
 * @return none
 * @author Cao Xin
 * @date 2025-04-03
 */
void draw_border(cv::Mat& image) {
    if (image.empty()) return;
    // 画矩形框
    cv::rectangle(image, cv::Point(0, 0), cv::Point(image.cols - 1, image.rows - 1), cv::Scalar(0), 10);
}

/**
 * @brief 迷宫法自适应域值扫线
 * @param img 输入的灰度图像
 * @param start 起始点坐标
 * @param block_size 自适应阈值的局部区域大小，默认为 7
 * @param clip_value 阈值偏移量，类似于曝光
 * @param max_points 最大提取点数
 * @return line_result 扫线结果
 * @author Cao Xin (adapted)
 * @date 2025-05-02
 */
line_result find_lines(cv::Mat img, cv::Point start, int block_size, int clip_value, int max_points) {
    cv::Mat temp = cv::Mat::zeros(img.size(), CV_8UC1);
    // 入参验证
    if (img.empty() || img.channels() != 1) {
        throw std::runtime_error("Input image must be a non-empty grayscale image.");
    }
    if (block_size < 3 || block_size % 2 == 0) {
        throw std::runtime_error("block_size must be an odd number >= 3.");
    }
    if (max_points <= 0) {
        throw std::runtime_error("max_points must be positive.");
    }
    if (start.x < 0 || start.x >= img.cols || start.y < 0 || start.y >= img.rows) {
        throw std::runtime_error("Starting point is out of image bounds.");
    }

    draw_border(img);

    // 方向定义：0:上, 1:右, 2:下, 3:左
    const int dir_front[4][2] = {{0, -1}, {1, 0}, {0, 1}, {-1, 0}};
    const int dir_frontleft[4][2] = {{-1, -1}, {1, -1}, {1, 1}, {-1, 1}};
    const int dir_frontright[4][2] = {{1, -1}, {1, 1}, {-1, 1}, {-1, -1}};

    line_result result;
    result.left.clear();
    result.right.clear();
    result.center.clear();

    // 计算局部区域半径
    int half = block_size / 2;

    // 左右边线巡线
    // mode-true 左手迷宫法
    // mode-false 右手迷宫法
    for (bool mode : {true, false}) {
        std::vector<cv::Point>& current_line = mode ? result.left : result.right;

        int x = start.x;
        int y = start.y;
        otsu_binarize(img, img, {0, y - 5}, {img.cols, y + 5});
        while(x < img.cols && x >= 0){
            int val = img.at<uchar>(y, x);
            if(val > 0){
                x += mode ? -1 : 1;
            }else{
                break;
            }
        }

        int dir = 0;
        int turn = 0;
        int step = 0;
        while (step < max_points && half < x && x < img.cols - half && half < y && y < img.rows - half && turn < 4) {
            // if(!mode){
            //     std::cout << x << ' ' << y << '\n';
            // }
            cv::Point pts0 = {std::max(x - half, 0), std::max(y - half, 0)};
            cv::Point pts1 = {std::min(x + half, img.cols), std::min(y + half, img.rows)};
            otsu_binarize(img, img, pts0, pts1);
            
            // 获取前方和侧方像素值
            int front_value = img.at<uchar>(y + dir_front[dir][1], x + dir_front[dir][0]);
            int side_value = mode
                             ? img.at<uchar>(y + dir_frontleft[dir][1], x + dir_frontleft[dir][0])
                             : img.at<uchar>(y + dir_frontright[dir][1], x + dir_frontright[dir][0]);
            
            if (front_value < 255) {
                // 前方是黑色就转弯
                dir = mode ? (dir + 1) % 4 : (dir + 3) % 4;
                turn++;
            } else if (side_value < 255) {
                // 侧方是黑色就前进
                x += dir_front[dir][0];
                y += dir_front[dir][1];
                current_line.push_back({x, y});
                step++;
                turn = 0;
            } else {
                // 前方和侧方都是是白色，朝侧方移动并转向
                x += mode ? dir_frontleft[dir][0] : dir_frontright[dir][0];
                y += mode ? dir_frontleft[dir][1] : dir_frontright[dir][1];
                dir = mode ? (dir + 3) % 4 : (dir + 1) % 4;
                current_line.push_back({x, y});
                step++;
                turn = 0;
            }
        }
    }
    cv::imshow("gray", img);
    return result;
}

/**
 * @brief 极大值和极小值限制
 * @param n 需要限制的值
 * @param lower 最小值
 * @param upper 最大值
 */
int clip(int n, int lower, int upper) {
    return std::max(lower, std::min(n, upper));
}

void blur_points(const std::vector<cv::Point>& pts_in, std::vector<cv::Point>& pts_out, int kernel) {
    assert(kernel % 2 == 1);
    int half = kernel / 2;
    int num = pts_in.size();
    pts_out.resize(num);
    for (int i = 0; i < num; i++) {
        double sum_x = 0;
        double sum_y = 0;
        double weight_sum = 0;
        for (int j = -half; j <= half; j++) {
            int index = clip(i + j, 0, num - 1);
            int weight = half + 1 - std::abs(j);
            sum_x += static_cast<double>(pts_in[index].x) * weight;
            sum_y += static_cast<double>(pts_in[index].y) * weight;
            weight_sum += weight;
        }
        pts_out[i].x = static_cast<int>(std::round(sum_x / weight_sum));
        pts_out[i].y = static_cast<int>(std::round(sum_y / weight_sum));
    }
}

// 点集等距采样
void resample_points(const std::vector<cv::Point>& pts_in, std::vector<cv::Point>& pts_out, float dist) {
    if (pts_in.empty()) {
        pts_out.clear();
        return;
    }

    pts_out.clear();
    pts_out.push_back(pts_in[0]);
    float remain = 0.0f;
    for (size_t i = 0; i < pts_in.size() - 1; ++i) {
        cv::Point p0 = pts_in[i];
        cv::Point p1 = pts_in[i + 1];
        float dx = static_cast<float>(p1.x - p0.x);
        float dy = static_cast<float>(p1.y - p0.y);
        float segment_length = std::sqrt(dx * dx + dy * dy);
        dx /= segment_length;
        dy /= segment_length;

        while (remain < segment_length) {
            float next_x = static_cast<float>(p0.x) + dx * remain;
            float next_y = static_cast<float>(p0.y) + dy * remain;
            pts_out.push_back(cv::Point(std::round(next_x), std::round(next_y)));
            segment_length -= remain;
            remain = dist;
        }
        remain -= segment_length;
    }
    // Ensure the last point is included
    if (!pts_in.empty() && (pts_out.empty() || pts_out.back() != pts_in.back())) {
        pts_out.push_back(pts_in.back());
    }
}

// 点集局部角度变化率
void get_line_angle(const std::vector<cv::Point>& pts_in, std::vector<float>& angle_out, int dist) {
    int num = pts_in.size();
    angle_out.resize(num);
    for (int i = 0; i < num; i++) {
        if (i <= 0 || i >= num - 1) {
            angle_out[i] = 0.0f;
            continue;
        }
        cv::Point p_prev = pts_in[clip(i - dist, 0, num - 1)];
        cv::Point p_curr = pts_in[i];
        cv::Point p_next = pts_in[clip(i + dist, 0, num - 1)];

        float dx1 = static_cast<float>(p_curr.x - p_prev.x);
        float dy1 = static_cast<float>(p_curr.y - p_prev.y);
        float dn1 = std::sqrt(dx1 * dx1 + dy1 * dy1);

        float dx2 = static_cast<float>(p_next.x - p_curr.x);
        float dy2 = static_cast<float>(p_next.y - p_curr.y);
        float dn2 = std::sqrt(dx2 * dx2 + dy2 * dy2);

        float c1 = dx1 / dn1;
        float s1 = dy1 / dn1;
        float c2 = dx2 / dn2;
        float s2 = dy2 / dn2;

        angle_out[i] = std::atan2(c1 * s2 - c2 * s1, c2 * c1 + s2 * s1);
    }
}

// 角度变化率非极大抑制
void filter_line_angle(const std::vector<float>& angle_in, std::vector<float>& angle_out, int kernel) {
    assert(kernel % 2 == 1);
    int half = kernel / 2;
    int num = angle_in.size();
    angle_out.resize(num);
    for (int i = 0; i < num; i++) {
        angle_out[i] = angle_in[i];
        for (int j = -half; j <= half; j++) {
            int index = clip(i + j, 0, num - 1);
            if (std::abs(angle_in[index]) > std::abs(angle_out[i])) {
                angle_out[i] = 0.0f;
                break;
            }
        }
    }
}