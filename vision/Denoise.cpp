#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

// 2x2 最小池化
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




// 图像降噪
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

int clip(int n, int lower, int upper) {
    return std::max(lower, std::min(n, upper));
}

// 点集三角滤波
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
void local_angle_points(const std::vector<cv::Point>& pts_in, std::vector<float>& angle_out, int dist) {
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
void nms_angle(const std::vector<float>& angle_in, std::vector<float>& angle_out, int kernel) {
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