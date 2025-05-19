#ifndef _VISON_H_
#define _VISON_H_

/**
 * @file Vision.cpp
 * @brief 图像处理主要操作头文件
 * @author Cao Xin
 * @date 2025-04-13
 */

#include "Bin.h"
#include "Track.h"
#include "Denoise.h"
#include "Line.h"
#include "Perspective.h"
#include "Ring.h"

#define VISION_DEBUG 1

typedef struct{
    int center;
    ElementType type;
}vision_result;

vision_result process_img(cv::Mat frame);
void draw_line(std::vector<int> line, cv::Mat& image);
#endif