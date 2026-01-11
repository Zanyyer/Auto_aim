#ifndef AUTO_AIM_SOLVER_HPP
#define AUTO_AIM_SOLVER_HPP

#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include "type.hpp"

namespace auto_aim 
{
class Solver 
{
public:
    // ======= 传入相机标定文件路径 ======= //
    explicit Solver(const std::string& path);
    
    // ======= PnP解算 ================= //
    bool solve(Armor& armor) const;

    // ======= 计算距离 ================= //
    float calculateDistanceToCenter(const cv::Point2f& img_points);

    // ======= 得到参数的接口 =========== // 
    const cv::Mat& getCameraMatrix() const { return camera_matrix_; }
    const cv::Mat& getDistCoeffs() const { return dist_coeffs_; }

    // ======= 投影函数: 将 3D obj pts 投影到像素面 ======= //
    void projectPoints(const std::vector<cv::Point3f>& obj_pts,
                       const cv::Vec3d& rvec,
                       const cv::Vec3d& tvec,
                       std::vector<cv::Point2f>& img_pts) const;

    // 根据 armor 类型直接投影装甲四角点
    void projectArmorPoints(const Armor& armor,
                            const cv::Vec3d& rvec,
                            const cv::Vec3d& tvec,
                            std::vector<cv::Point2f>& img_pts) const;

private:
    // ======= 装甲板参数 ============== // m   
    static constexpr float SMALL_ARMOR_WIDTH  = 135e-3f;
    static constexpr float SMALL_ARMOR_HEIGHT = 55e-3f;
    static constexpr float LARGE_ARMOR_WIDTH  = 230e-3f;
    static constexpr float LARGE_ARMOR_HEIGHT = 55e-3f;

    // ======= 相机参数 ================ //
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;

    std::vector<cv::Point3f> small_armor_pts_;
    std::vector<cv::Point3f> big_armor_pts_;
};

} // namespace auto_aim
#endif // AUTO_AIM_SOLVER_HPP
