#ifndef AUTO_AIM_TRACKER_HPP
#define AUTO_AIM_TRACKER_HPP

#include <opencv2/core.hpp>
#include <chrono>

namespace auto_aim {
class ArmorTracker {
public:
    ArmorTracker();

    void reset(const cv::Point3f& pos,
               std::chrono::steady_clock::time_point t);

    bool update(const cv::Point3f& pos,
                std::chrono::steady_clock::time_point t);

    void predict(double dt);

    void drawPredict(
        cv::Mat& img, 
        const cv::Mat& camera_matrix, 
        const cv::Mat& dist_coeffs, 
        cv::Scalar colors
    ) const;

    cv::Point3f getPrePosition() {return pred_position_; }; 

private:
    cv::Mat x_;   // 6x1 卡尔曼滤波的状态向量 [x, y, z, vx, vy, vz]^T
    cv::Mat P_;   // 6x6 状态协方差矩阵，表示状态估计的不确定性（对角线值越大，不确定性越高） 
    cv::Mat Q_;   // 6x6 过程噪声协方差矩阵，表示模型误差
    cv::Mat R_;   // 3x3 观测噪声协方差矩阵，表示测量误差

    bool initialized_{false};
    std::chrono::steady_clock::time_point last_time_; // 上一次更新的时间戳，用于计算时间差 dt
    cv::Point3f pred_position_;
};

} // namespace auto_aim
#endif // AUTO_AIM_TRACKER_HPP