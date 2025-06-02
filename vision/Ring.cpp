#include "Ring.h"
#include "Line.h"
#include "Track.h"
#include <opencv2/core/types.hpp>
#include <vector>

/**
 * @file Ring.cpp
 * @brief 圆环边线计算相关
 * @author Cao Xin
 * @date 2025-06-02
 */

void calc_right_ring_target(track_result &track, ElementType type);
void calc_left_ring_target(track_result &track, ElementType type);
/**
 * @brief 确定最终拟合的中线以及预锚点的 Y 值
 */
void calc_ring_target(track_result &track, ElementType type) {
    if (type == L_RING_BEGIN || type == L_RING_IN || type == L_RING_OUT) {
        calc_right_ring_target(track, type);
    }
    if (type == R_RING_BEGIN || type == R_RING_IN || type == R_RING_OUT) {
        calc_left_ring_target(track, type);
    }

    cv::Size size = track.left.frame_size;
    track.target.y = size.height - 15;
}

/**
 * @brief 右圆环处理
 */
void calc_right_ring_target(track_result &track, ElementType type) {
    if (type == R_RING_BEGIN) {
        track.center = track.left.center;
    } else if (type == R_RING_IN) {
        track.center = track.right.center;
    } else if (type == R_RING_OUT) {
        track.center = track.left.center;
    }
}

/**
 * @brief 左圆环处理
 */
void calc_left_ring_target(track_result &track, ElementType type) {
    if (type == L_RING_BEGIN) {
        track.center = track.right.center;
    } else if (type == L_RING_IN) {
        track.center = track.left.center;
    } else if (type == L_RING_OUT) {
        track.center = track.right.center;
    }
}

/**
 * @brief 计算是否已经进入圆环
 * @return 是否进入圆环
 */
bool calc_is_ring_in(track_result &track) {
    cv::Size size = track.left.frame_size;
    int height = size.height;
    int width = size.width;

    std::vector<int> left(-1, height), right(-1, height);
    for (cv::Point pt : track.left.line) {
        if (left[pt.y] == -1) {
            continue;
        }
        left[pt.y] = pt.x;
    }
    for (cv::Point pt : track.right.line) {
        if (left[pt.y] == -1) {
            continue;
        }
        right[pt.y] = pt.x;
    }

    int min_idx = -1;
    int min_dist = -1;
    for (int i = 0; i < height; i++) {
        if (left[i] == -1 || right[i] == -1) {
            continue;
        }
        int dist = abs(left[i] - right[i]);
        if (min_idx == -1) {
            min_idx = i;
            min_dist = dist;
        } else if (dist < min_dist){
            min_idx = i;
        }
    }
    
    return min_idx == -1 ? false : min_idx < track.target.y;
}