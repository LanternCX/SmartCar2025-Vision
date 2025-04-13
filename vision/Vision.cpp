#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

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

void process_img(cv::Mat frame){
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
}

