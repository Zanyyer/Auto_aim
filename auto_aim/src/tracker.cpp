#include "tracker.hpp"
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

namespace auto_aim {

ArmorTracker::ArmorTracker() {
    x_ = cv::Mat::zeros(6, 1, CV_64F);     
    P_ = cv::Mat::eye(6, 6, CV_64F) * 1e-1; 
    Q_ = cv::Mat::eye(6, 6, CV_64F) * 1e-3;
    R_ = cv::Mat::eye(3, 3, CV_64F) * 1e-2; 
}

void ArmorTracker::reset(const cv::Point3f& position,
                         std::chrono::steady_clock::time_point t)
{
    x_.at<double>(0) = position.x; 
    x_.at<double>(1) = position.y; 
    x_.at<double>(2) = position.z; 
    x_.at<double>(3) = 0;          
    x_.at<double>(4) = 0;      
    x_.at<double>(5) = 0;      

    P_ = cv::Mat::eye(6, 6, CV_64F) * 1e-1; // 重置协方差矩阵
    last_time_ = t;
    initialized_ = true;
}

bool ArmorTracker::update(const cv::Point3f& position,
                          std::chrono::steady_clock::time_point t) 
{
    if (!initialized_) {
        reset(position, t);
        return false;
    }
    // d(t)  
    double dt = std::chrono::duration<double>(t - last_time_).count();
    last_time_ = t;

    // F/A (状态转移矩阵)
    cv::Mat F = cv::Mat::eye(6, 6, CV_64F);
    F.at<double>(0,3) = dt;  
    F.at<double>(1,4) = dt; 
    F.at<double>(2,5) = dt; 
    
    // predict                             
    x_ = F * x_;              // 先验估计
    P_ = F * P_ * F.t() + Q_; // 先验误差协方差矩阵

    // update
    cv::Mat z = (cv::Mat_<double>(3,1) << position.x, position.y, position.z);

    cv::Mat H = cv::Mat::zeros(3, 6, CV_64F); // 转换矩阵
    H.at<double>(0,0) = 1; 
    H.at<double>(1,1) = 1;
    H.at<double>(2,2) = 1;

    cv::Mat y = z - H * x_;
    cv::Mat S = H * P_ * H.t() + R_;
    cv::Mat K = P_ * H.t() * S.inv(); 
                                      
    x_ = x_ + K * y;
    cv::Mat I = cv::Mat::eye(6, 6, CV_64F);
    P_ = (I - K * H) * P_;

    return true;
}

void ArmorTracker::predict(double dt) {
    cv::Mat xp = x_.clone();
    xp.at<double>(0) += xp.at<double>(3) * dt;
    xp.at<double>(1) += xp.at<double>(4) * dt;
    xp.at<double>(2) += xp.at<double>(5) * dt;

    pred_position_ = {
        static_cast<float>(xp.at<double>(0)),
        static_cast<float>(xp.at<double>(1)),
        static_cast<float>(xp.at<double>(2))
    };
}

void ArmorTracker::drawPredict(cv::Mat& img,
                               const cv::Mat& camera_matrix,
                               const cv::Mat& dist_coeffs,
                               cv::Scalar color           ) const
{
    if (!initialized_) return;

    //  3D → 2D 投影
    std::vector<cv::Point3f> obj_pts = { pred_position_ };
    std::vector<cv::Point2f> img_pts;

    cv::projectPoints(
        obj_pts,
        cv::Vec3d::zeros(),   // rvec
        cv::Vec3d::zeros(),   // tvec
        camera_matrix,
        dist_coeffs,
        img_pts
    );

    if (img_pts.empty()) return;

    // 3. 绘制预测点 (不显示文字标签)
    cv::circle(img, img_pts[0], 2, color, 2);

}

} // namespace auto_aim
