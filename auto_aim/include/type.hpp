#ifndef AUTO_AIM_TYPE_HPP
#define AUTO_AIM_TYPE_HPP

#include <vector>
#include <opencv2/core.hpp>

namespace auto_aim {

const int RED = 0;
const int BLUE = 1;

enum ArmorType {
    UNKNOE = -1,
    SMALL = 0,
    LARGE = 1,
};

// ============ 灯条 ================ //
struct Light : public cv::RotatedRect 
{
    Light() = default;
    explicit Light(cv::RotatedRect box) : cv::RotatedRect(box)
    {
        cv::Point2f p[4];
        box.points(p);
        std::sort(p, p+4, 
            [](const cv::Point2f& a, const cv::Point2f& b) {return a.y < b.y;}); 
        
        top = (p[0] + p[1]) * 0.5;
        bottom = (p[2] + p[3]) * 0.5;

        length = cv::norm(top - bottom);
        width = std::min(box.size.width, box.size.height);
        
        angle = std::atan2(std::abs(top.x - bottom.x), std::abs(top.y - bottom.y));
        angle = angle / CV_PI * 180;  // 弧度(rad) -> 角度(deg)
        
        center = box.center;
    }

    float angle;
    float length;
    float width;

    cv::Point2f center;
    cv::Point2f top, bottom;
    int color;
};

// =========== 装甲板 ============ //
struct Armor 
{
    Armor() = default;
    explicit Armor(const Light& l1, const Light& l2) 
    {
        if(l1.center.x < l2.center.x){
            left_light = l1, right_light = l2;
        } else {
            left_light = l2, right_light = l1;
        }
        center = (left_light.center + right_light.center) * 0.5;
    }
    // ======= Detector ======= //
    ArmorType type;
    cv::Point2f center;
    Light left_light, right_light;

    // ======= Solver 阶段 ========= //
    std::vector<cv::Point2f> points;   // 4个角点（2D）
    cv::Point3f position;              // PnP 解算结果,单位 m

    cv::Vec3d rvec; // 旋转向量 
    cv::Vec3d tvec; // 平移向量
};

} // namespace auto_aim
#endif // AUTO_AIM_TYPE_HPP