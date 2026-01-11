#ifndef AUTO_AIM_DETECTOR_HPP
#define AUTO_AIM_DETECTOR_HPP

#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include "type.hpp"

namespace auto_aim 
{
class Detector
{
public:
    struct Params
    {
        // // ======= 预处理参数 =========== //
        int binary_threshold;       // 二值化阈值
        cv::Size kernel_size = cv::Size(1,1); 
        int detect_color;

        // ======= 灯条参数 ============ //
        struct LightParam 
        {
            float min_ratio;      // 最小灯条长宽比
            float max_ratio;      // 最大灯条长宽比
            float max_angle;      // 最大灯条倾斜角
        } light_params;

        // ======= 判断装甲板参数 ======= //
        struct ArmorParam 
        {
            float min_small_center_dist;     // 最小 小装甲板 中心距
            float max_small_center_dist;     // 最大 小装甲板 中心距
            float min_large_center_dist;     // 最小 大装甲板 中心距
            float max_large_center_dist;     // 最大 大装甲板 中心距
            float max_angle_diff;            // 最大灯条角度差
        } armor_params;
        
        // ======= 读取配置 ========== //
        bool loadParams(const std::string& path);

    };

public:
    // ======= 初始化参数 ========= //
    explicit Detector(const Params& param);

    // ======= 主检测接口 ========= //
    std::vector<Armor> detect(const cv::Mat& input);

    // ======= 绘制调试信息 ======= //
    void drawDebug(cv::Mat& img, cv::Scalar color) const;



private:
    // ======= 内部处理流程 ======= //
    cv::Mat preprocess(const cv::Mat& input);
    std::vector<Light> findLights(const cv::Mat& rgb_img, const cv::Mat& bin);
    std::vector<Armor> matchLights(const std::vector<Light>& lights);
    bool containLight(
        const Light & light_1, const Light & light_2, const std::vector<Light> & lights);
    
    // ======= 判定函数 ========== //
    bool isLight(const Light& light) const;
    bool isArmor( Armor& armor) const ;

private:
    Params param_;
    
    cv::Mat processed_img_;
    std::vector<Light> lights_;
    std::vector<Armor> armors_;
};

} // namespace auto_aim
#endif // AUTO_AIM_DETECTOR_HPP
