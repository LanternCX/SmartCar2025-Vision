#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>

#include <algorithm>
#include <iostream>
#include <vector>

#include "Line.h"
#include "Vision.h"
#include "Math.h"
#include "Debug.h"

/**
 * @file Track.cpp
 * @brief 扫线相关操作
 * @author Cao Xin
 * @date 2025-04-03
 */

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
 * @param block_size 自适应阈值的局部区域大小, 默认 7, 奇数
 * @param max_points 最大提取点数, 默认 1000
 * @return line_result 扫线结果
 * @author Cao Xin
 * @date 2025-05-02
 */
track_result find_lines(cv::Mat img, cv::Point start, int block_size, int max_points) {
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

    track_result result;
    result.left.line.clear();
    result.right.line.clear();

    // 计算局部区域半径
    int half = block_size / 2;

    // 左右边线巡线
    // 迷宫法简介: https://zhuanlan.zhihu.com/p/391392970

    // mode-true 左手迷宫法
    // mode-false 右手迷宫法
    for (bool mode : {true, false}) {
        std::vector<cv::Point>& current_line = mode ? result.left.line : result.right.line;
        current_line.clear();

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
            cv::Point pts0 = {std::max(x - half, 0), std::max(y - half, 0)};
            cv::Point pts1 = {std::min(x + half, img.cols), std::min(y + half, img.rows)};

            // denoise(img, cv::Point(x, y), half);
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
    return result;
}

/**
 * @brief 判断赛道元素类型
 */
ElementType get_element_type(track_result &track) {
    // l, r
    std::array<int, 2> corner_cnt;
    line_result left = track.left;
    line_result right = track.right;

    // 镜像右边线使得逻辑统一
    right.line = mirror_line(right.line, right.frame_size);
    // std::reverse(right.line.begin(), right.line.end()); 
    
    corner_cnt[0] = get_corner_count(trans_line(left.line, left.frame_size));
    corner_cnt[1] = get_corner_count(trans_line(right.line, right.frame_size));

    debug(corner_cnt);
    
    if (corner_cnt[0] == 2 && corner_cnt[1] == 2) {
        track.type = CROSS_PRE;
        return CROSS_PRE;
    }

    if (corner_cnt[0] == 2 && corner_cnt[1] < 2) {
        track.type = L_RING_PRE;
        return L_RING_PRE;
    }

    if (corner_cnt[0] < 2 && corner_cnt[1] == 2) {
        track.type = R_RING_PRE;
        return R_RING_PRE;
    }
    
    line_params left_res = fit_line(left.line);
    line_params right_res = fit_line(right.line);

    if (left_res.slope > 0.15) {
        track.type = R_CURVE;
        return R_CURVE;
    }

    if (right_res.slope < -0.15) {
        track.type = L_CURVE;
        return L_CURVE;
    }

    track.type = LINE;
    return LINE;
}