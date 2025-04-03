#include <opencv2/core/hal/interface.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include "Line.h"
using namespace std;

cv::Mat lineDetection(cv::Mat binary, cv::Mat src) {
    // 图像尺寸
    int width = binary.cols;
    int height = binary.rows;

    // 用于存放左右边界点
    vector<cv::Point> leftPoints;
    vector<cv::Point> rightPoints;

    // 选取扫描起始行（例如离底部较近的行）
    int startRow = height - 10;
    cv::line(src, cv::Point(0, startRow), cv::Point(width - 1, startRow), cv::Scalar(0, 255, 255), 5);
        
    int midCol = width / 2;

    // 寻找左边起点：从中间向左扫描，找到黑白跳变点
    cv::Point leftStart = cv::Point(width, startRow);
    bool foundLeft = false;
    for (int i = midCol; i > 0; i--) {
        if (binary.at<uchar>(startRow, i) == 255 && binary.at<uchar>(startRow, i - 1) == 0) {
            leftStart = cv::Point(i, startRow);
            foundLeft = true;
            break;
        }
    }

    // 寻找右边起点：从中间向右扫描，找到黑白跳变点
    cv::Point rightStart = cv::Point(0, startRow);
    bool foundRight = false;
    for (int i = midCol; i < width - 1; i++) {
        if (binary.at<uchar>(startRow, i) == 255 && binary.at<uchar>(startRow, i + 1) == 0) {
            rightStart = cv::Point(i, startRow);
            foundRight = true;
            break;
        }
    }

    // 丢失左边线就向上寻找
    if(!foundLeft){
        for(int i = startRow; i > 0; i--){
            if(binary.at<uchar>(i, 0) == 255 && binary.at<uchar>(i - 1, 0) == 0){
                leftStart = cv::Point(0, i - 1);
                cout << "found left idx: " << i << '\n';
                break;
            }
        }
    }

    // 丢失右边线就向上寻找
    if(!foundRight){
        for(int i = startRow; i > 0; i--){
            if(binary.at<uchar>(i, width - 1) == 255 && binary.at<uchar>(i - 1, width - 1) == 0){
                rightStart = cv::Point(width - 1, i - 1);
                cout << "found right idx: " << i << '\n';
                break;
            }
        }
    }

    // 初始化扫描中心点
    cv::Point leftCenter = leftStart;
    cv::Point rightCenter = rightStart;
    leftPoints.push_back(leftCenter);
    rightPoints.push_back(rightCenter);

    // 定义左右两边的八邻域偏移数组
    // 左边逆时针
    int seedsL[8][2] = {
        {0,  1}, {-1, 1}, {-1, 0}, {-1, -1},
        {0, -1}, {1, -1}, {1,  0}, {1,  1}
    };
    // 右边顺时针
    int seedsR[8][2] = {
        {0,  1}, {1, 1}, {1,  0}, {1, -1},
        {0, -1}, {-1, -1}, {-1, 0}, {-1, 1}
    };

    int maxIter = 1000;
    while (maxIter--) {
        // 左边扫描
        vector<cv::Point> leftCandidates;
        for (int i = 0; i < 8; i++) {
            // 当前八领域检查的点
            int nowX = leftCenter.x + seedsL[i][0];
            int nowY = leftCenter.y + seedsL[i][1];
            if (nowX < 0 || nowX >= width || nowY < 0 || nowY >= height){
                continue;
            }

            // 下一个八领域检查的点
            int nextIdx = (i + 1) % 8;
            int nextX = leftCenter.x + seedsL[nextIdx][0];
            int nextY = leftCenter.y + seedsL[nextIdx][1];
            if (nextX < 0 || nextX >= width || nextY < 0 || nextY >= height){
                continue;
            }

            // 检查黑白跳变
            if (binary.at<uchar>(nowY, nowX) == 0 && binary.at<uchar>(nextY, nextX) == 255) {
                leftCandidates.push_back(cv::Point(nowX, nowY));
            }
        }
        // 选候选点中最靠上的点
        if (!leftCandidates.empty()) {
            cv::Point best = leftCandidates[0];
            for (auto pt : leftCandidates) {
                if (pt.y < best.y)
                    best = pt;
            }
            leftCenter = best;
            leftPoints.push_back(leftCenter);
        }

        // 右边扫描
        vector<cv::Point> rightCandidates;
        for (int i = 0; i < 8; i++) {
            int nowX = rightCenter.x + seedsR[i][0];
            int nowY = rightCenter.y + seedsR[i][1];
            if (nowX < 0 || nowX >= width || nowY < 0 || nowY >= height) {
                continue;
            }

            int nextIdx = (i + 1) % 8;
            int nextX = rightCenter.x + seedsR[nextIdx][0];
            int nextY = rightCenter.y + seedsR[nextIdx][1];
            if (nextX < 0 || nextX >= width || nextY < 0 || nextY >= height) {
                continue;
            }

            if (binary.at<uchar>(nowY, nowX) == 0 && binary.at<uchar>(nextY, nextX) == 255) {
                rightCandidates.push_back(cv::Point(nowX, nowY));
            }
        }
        // 选候选点中最靠上的点
        if (!rightCandidates.empty()) {
            cv::Point best = rightCandidates[0];
            for (auto pt : rightCandidates) {
                if (pt.y < best.y)
                    best = pt;
            }
            rightCenter = best;
            rightPoints.push_back(rightCenter);
        }

        // 判断左右边线是否相遇
        if (abs(leftCenter.x - rightCenter.x) < 2 && abs(leftCenter.y - rightCenter.y) < 2) {
            break;
        }
    }

    // 画左右边线
    for (size_t i = 0; i < leftPoints.size(); i++) {
        cv::circle(src, leftPoints[i], 2, cv::Scalar(0, 0, 255), -1);
    }
    for (size_t i = 0; i < rightPoints.size(); i++) {
        cv::circle(src, rightPoints[i], 2, cv::Scalar(255, 0, 0), -1);
    }
    for (size_t i = 0; i < leftPoints.size(); i++) {
        cv::circle(src, {(leftPoints[i].x + rightPoints[i].x) / 2, (leftPoints[i].y + rightPoints[i].y) / 2}, 2, cv::Scalar(0, 255, 0), -1);
    }

    return src;
}