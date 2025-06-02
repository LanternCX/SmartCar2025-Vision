#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

#include "Cross.h"
#include "Line.h"
#include "Vision.h"

/**
 * @brief 计算十字
 */
void calc_cross_target(track_result &track, ElementType type) {
    track.center = track.left.center;
    cv::Size size = track.left.frame_size;
    track.target.y = size.height - 30;
}