#include <vector>
#include <opencv2/opencv.hpp>

#include "Math.h"

/**
 * @file Math.cpp
 * @brief 数学相关工具
 * @author Cao Xin
 * @date 2025-04-05
 */

/**
 * @brief 限幅
 * @param a 输入值
 * @param min 最小值
 * @param max 最大值
 * @return 限幅之后的数据
 * @author Cao Xin
 * @date 2025-04-05
 */
float clip(float a, float min, float max){
    if(a < min){
        return min;
    }
    if(a > max){
        return max;
    }
    return a;
}

/**
 * @brief 取最小值
 * @param a
 * @param b
 * @return a 和 b 的最小值
 * @author Cao Xin
 * @date 2025-04-05
 */
float min(float a, float b){
    return a > b ? b : a;
}

/**
 * @brief 取最大值
 * @param a
 * @param b
 * @return a 和 b 的最大值
 * @author Cao Xin
 * @date 2025-04-05
 */
float max(float a, float b){
    return a < b ? b : a;
}

/**
 * @brief 最小二乘法拟合直线
 * @param points 输入的点集
 * @return 拟合后的直线参数
 * @author Cao Xin
 * @date 2025-04-13
 */
line_params fit_line(const std::vector<cv::Point>& points) {
    line_params params = {0.0f, 0.0f, 0.0f, false};

    if (points.empty()) {
        return params;
    }

    // 计算统计量
    float sum_x = 0, sum_y = 0, sum_xy = 0, sum_xx = 0;
    for (const auto& p : points) {
        sum_x += p.x;
        sum_y += p.y;
        sum_xy += p.x * p.y;
        sum_xx += p.x * p.x;
    }
    float n = static_cast<float>(points.size());
    float mean_x = sum_x / n;
    float mean_y = sum_y / n;

    // 计算斜率
    float denominator = sum_xx - n * mean_x * mean_x;
    // 垂直线
    if (std::abs(denominator) < 1e-6) {
        params.is_vertical = true;
        params.c = mean_x;
    } else {
        params.slope = (sum_xy - n * mean_x * mean_y) / denominator;
        params.intercept = mean_y - params.slope * mean_x;
    }

    return params;
}

/**
 * @brief 计算单位向量
 * @param v 输入向量
 * @return 计算出的单位向量
 * @author Cao Xin
 * @date 2025-06-01
 */
cv::Point2f normalize(const cv::Point2f& v) {
    float length = std::sqrt(v.x * v.x + v.y * v.y);
    if (length == 0.0f) return cv::Point2f(0.0f, 0.0f);
    return cv::Point2f(v.x / length, v.y / length);
}

/**
 * @brief 计算向量长度
 * @param v 输入向量
 * @return 计算处的向量长度
 * @author Cao Xin
 * @date 2025-06-01
 */
static float length(const cv::Point2f& v) {
    return std::sqrt(v.x * v.x + v.y * v.y);
}

/**
 * @brief 沿切线方向平移点集
 * @param line 输入的曲线点集
 * @param dist 平移的距离
 * @return 平移后的点集 
 * @author Cao Xin
 * @date 2025-06-01
 */
std::vector<cv::Point> shift_line(
    const std::vector<cv::Point>& line,
    float shift_distance
) {
    size_t n = line.size();
    if (n < 2) {
        // 少于两个点时，无法确定切线，直接返回空
        return {};
    }

    // 先把所有点转换到浮点版本，方便计算
    std::vector<cv::Point2f> pts(n);
    for (size_t i = 0; i < n; ++i) {
        pts[i] = cv::Point2f(line[i].x, line[i].y);
    }

    std::vector<cv::Point> result(n);

    // 记录上一个非零切线方向的单位切线，防止连续相同点导致切线为零
    cv::Point2f prev_unit_tan(0, 0);

    for (size_t i = 0; i < n; ++i) {
        // 1) 计算当前顶点 i 处的切线方向
        //    对于内点，用中心差分：P[i+1] - P[i-1]
        //    对于第一个点，用前向差分：P[1] - P[0]
        //    对于最后一个点，用后向差分：P[n-1] - P[n-2]
        cv::Point2f raw_tan(0.f, 0.f);
        if (i == 0) {
            raw_tan = pts[1] - pts[0];
        } else if (i == n - 1) {
            raw_tan = pts[n - 1] - pts[n - 2];
        } else {
            raw_tan = pts[i + 1] - pts[i - 1];
        }

        // 2) 归一化为单位切线
        cv::Point2f unit_tan = normalize(raw_tan);
        if (length(unit_tan) < 1e-6f) {
            // 如果当前差分向量几乎为零（相邻点重合），则继续使用上一个非零切线方向
            unit_tan = prev_unit_tan;
        } else {
            prev_unit_tan = unit_tan;
        }

        // 3) 计算法线方向：逆时针旋转 90° 得到“左侧法线”
        //    (dx, dy) -> (-dy, dx)
        cv::Point2f unit_norm(-unit_tan.y, unit_tan.x);

        // 4) 当前点沿法线偏移 shift_distance
        result[i] = pts[i] + unit_norm * shift_distance;
    }

    return result;
}
