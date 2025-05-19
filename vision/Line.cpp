#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

#include "Line.h"
#include "Vision.h"
#include "Math.h"

/**
 * @file Line.cpp
 * @brief 边线处理相关操作
 * @author Cao Xin
 * @date 2025-05-03
 */

/**
 * @brief 点集三角滤波
 * @param pts_in 输入点集
 * @param pts_out 输出点集
 * @param kernel 滤波核大小 奇数
 * @return none
 * @author Cao Xin
 * @date 2025-05-03
 */
void filter_points(const std::vector<cv::Point>& pts_in, std::vector<cv::Point>& pts_out, int kernel) {
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

/**
 * @brief 点集等距采样
 * @param pts_in 输入点集
 * @param pts_out 输出点集
 * @param dist 采样距离
 * @return none
 * @author Cao Xin
 * @date 2025-05-03
 */
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

/**
 * @brief 局部角度变化率
 * @param pts_in 输入点集
 * @param angle_out 计算得的变化率集
 * @param angle_dist 斜率采样距离
 * @param sample_dist 等距采样距离
 * @return none
 * @author Cao Xin
 * @date 2025-05-03
 */
void get_line_slope(const std::vector<cv::Point>& pts_in, std::vector<float>& angle_out, int angle_dist, int sample_dist) {
    const float epsilon = 1e-6f;
    int dist = angle_dist / sample_dist;
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

        if (dn1 < epsilon || dn2 < epsilon) {
            angle_out[i] = 0.0f;
            continue;
        }

        float c1 = dx1 / dn1;
        float s1 = dy1 / dn1;
        float c2 = dx2 / dn2;
        float s2 = dy2 / dn2;

        angle_out[i] = std::atan2(c1 * s2 - c2 * s1, c2 * c1 + s2 * s1);
    }
}

/**
 * @brief 角度变化率非极大值抑制
 * @param angle_in 输入变化率集
 * @param angle_out 抑制后的变化率集
 * @param kernel 滤波核大小
 * @return none
 * @author Cao Xin
 * @date 2025-05-03
 */
void  filter_line_slope(const std::vector<float>& angle_in, std::vector<float>& angle_out, int kernel) {
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

/**
 * @brief 从角度变化率重建点集
 * @param angle_in 输入的角度变化率
 * @param pts_out 输出点集
 * @param dist 每步移动的距离
 * @param origin 初始点坐标
 * @return none
 * @author Cao Xin
 * @date 2025-05-04
 */
void rebuild_line(const std::vector<float>& angle_in, std::vector<cv::Point>& pts_out, int dist, cv::Point origin) {
    int num = angle_in.size();
    pts_out.resize(num);
    pts_out[0] = origin;

    float angle = 0.0f;

    for (int i = 1; i < num; i++) {
        angle += angle_in[i];
        float dx = std::cos(angle) * dist;
        float dy = std::sin(angle) * dist;
        pts_out[i] = cv::Point(
            static_cast<int>(std::round(pts_out[i - 1].x + dx)),
            static_cast<int>(std::round(pts_out[i - 1].y + dy))
        );
    }
}

/**
 * @brief 通过角度变化率判断角点类型
 * @param src 左右边线的局部角度变化率
 * @return 计算出的角点变化率结果
 * @author Cao Xin
 * @date 2025-05-04
 */
std::vector<cv::Point> find_corners(const std::vector<cv::Point>& points, double angleThresholdDegrees) {
    std::vector<cv::Point> corners;

    for (size_t i = 1; i + 1 < points.size(); ++i) {
        cv::Point2f v1 = points[i] - points[i - 1];
        cv::Point2f v2 = points[i + 1] - points[i];

        double dot = v1.x * v2.x + v1.y * v2.y;
        double mag1 = std::sqrt(v1.x * v1.x + v1.y * v1.y);
        double mag2 = std::sqrt(v2.x * v2.x + v2.y * v2.y);

        if (mag1 < 1e-6 || mag2 < 1e-6) continue;

        double cosTheta = dot / (mag1 * mag2);
        cosTheta = clip(cosTheta, -1.0, 1.0);

        double angle = std::acos(cosTheta) * 180.0 / CV_PI;

        if (angle < 180.0 - angleThresholdDegrees) {
            corners.push_back(points[i]);
        }
    }

    return corners;
}

/**
 * @brief 重新采样到 line[y] = x 边线表示
 * @param line 边线点集
 * @param size 图像大小
 */
std::vector<int> trans_line(const std::vector<cv::Point> &line, const cv::Size &size) {
    int height = size.height;
    int width = size.width;
    std::vector<int> res(height, -1);
    for (cv::Point pts : line) {
        if (res[pts.y] != -1) {
            continue;
        }
        res[pts.y] = pts.x;
    }
    for (int &x : res) {
        if (x == -1) {
            x = width / 2;
        }
    }
    return res;
}

/**
 * @brief 角点数量判断
 * @param line 边线
 * @param threshold 角点阈值
 */
int get_corner_count(const std::vector<int> &line, const int &threshold) {
    int n = line.size();
    int dist = 15;
    int cnt = 0;
    // 从近到远找到第一个拐点
    for (int i = 0; i < n; i++) {
        int next = clip(i, 0, n - 1);
        if (abs(line[next] - line[i]) > threshold) {
            cnt++;
            break;
        }
    }
    // 从远到近找到第二个拐点
    for (int i = n - 1; i >= 0; i--) {
        int next = clip(i, 0, n - 1);
        if (abs(line[next] - line[i]) > threshold) {
            cnt++;
            break;
        }
    }
    return cnt;
}

/**
 * @brief 判断元素类型
 */
ElementType get_element_type(const line_result& line, const int ) {
    
}


/**
 * @brief 最小二乘法判断直线
 * @param line 边线
 * @param image 输入的图像
 * @return 是否为直线
 * @author Cao Xin
 * @date 2025-04-13
 */
bool is_line(const std::vector<cv::Point2f>& points, float threshold = 3.0) {
    // 点数太少默认为曲线
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