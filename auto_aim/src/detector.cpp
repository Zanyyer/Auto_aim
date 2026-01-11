#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <cmath>
#include <yaml-cpp/yaml.h>
#include "detector.hpp"
// #include <iostream>

namespace auto_aim{
Detector::Detector(const Params& param) : param_(param) {}

// 主检测接口
std::vector<Armor> Detector::detect(const cv::Mat& input) {
    if (input.empty()) return {};

    processed_img_ = preprocess(input);
    
    lights_ = findLights(input, processed_img_);
    
    armors_ = matchLights(lights_);
    
    return armors_;
}

// 图像预处理
cv::Mat Detector::preprocess(const cv::Mat& rgb_img) {
    cv::Mat gray, bin, morph;

    cv::cvtColor(rgb_img, gray, cv::COLOR_BGR2GRAY);

    cv::threshold(gray, bin, param_.binary_threshold, 255, cv::THRESH_BINARY);
    cv::Mat kernel = 
        cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3));

    // 腐蚀 膨胀
    cv::erode(bin, morph, kernel);
    cv::dilate(morph, morph, kernel);
    
    return morph;
}

// =========== 寻找灯条并初始化灯条 ========= //
std::vector<Light> Detector::findLights(const cv::Mat& rgb_img, const cv::Mat& bin)
{
    using std::vector;

    vector<vector<cv::Point>> contours; 
    vector<cv::Vec4i> hierarchy;
    cv::findContours(bin, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    vector<Light> detected_lights;        
    for(const auto& contour : contours){
        cv::RotatedRect r_rect = cv::minAreaRect(contour);
        Light light(r_rect);

        if (!isLight(light))   continue;
        
        auto rect = light.boundingRect();
        // Avoid assertion failed
        if (0 <= rect.x && 0 <= rect.width && rect.x + rect.width <= rgb_img.cols && 
            0 <= rect.y &&0 <= rect.height && rect.y + rect.height <= rgb_img.rows){
            int sum_red = 0, sum_blue = 0;
            auto roi = rgb_img(rect);
            for(int i = 0; i < roi.rows; i++){
                for(int j = 0; j < roi.cols; j++){
                    if (cv::pointPolygonTest(contour, cv::Point2f(j + rect.x, i + rect.y), false) >= 0) {
                        sum_blue += roi.at<cv::Vec3b>(i, j)[0];
                        sum_red  += roi.at<cv::Vec3b>(i, j)[2];
                    }
                }
            }
            light.color = sum_red > sum_blue ? RED : BLUE;
            detected_lights.emplace_back(light);
        }
    }
    return detected_lights;
} 

// ============ 判定是否为灯条 ================= //
bool Detector::isLight(const Light& light) const
{
    // 长宽比判断
    float ratio = light.length / light.width;
    if(ratio  < param_.light_params.min_ratio || 
        ratio > param_.light_params.max_ratio) {
        return false;
    }
    // 倾斜角判断
    if(light.angle > param_.light_params.max_angle){
        return false;
    }
    return true;
}

// =========== 配对灯条，形成装甲板 =========== //
std::vector<Armor> Detector::matchLights(const std::vector<Light>& lights)
{
    std::vector<Armor> armors;
    int lights_size = lights.size();
    for(int i = 0 ; i < lights_size - 1; ++i){
        for(int j = i + 1; j < lights_size; ++j){
            const Light& l1 = lights[i]; 
            const Light& l2 = lights[j];

            if (l1.color != param_.detect_color ||
                l2.color != param_.detect_color ) continue;

            if(containLight(l1, l2, lights)){
                continue;
            }
            auto armor = Armor(l1, l2);
            if(isArmor(armor)){
                armors.emplace_back(armor);
            }
        }
    } 
    return armors;
}

// ============= 储存灯条对 ========== //
bool Detector::containLight(const Light & light_1, const Light & light_2, 
                            const std::vector<Light> & lights)
{
    auto points = std::vector<cv::Point2f>{light_1.top, light_1.bottom, light_2.top, light_2.bottom};
    auto bounding_rect = cv::boundingRect(points);

    for (const auto & test_light : lights) {
        if (test_light.center == light_1.center || test_light.center == light_2.center) continue;

        if (
        bounding_rect.contains(test_light.top) || bounding_rect.contains(test_light.bottom) ||
        bounding_rect.contains(test_light.center)) {
        return true;
        }
    }
  return false;
}

// =========== 是否为装甲板 ============= //
bool Detector::isArmor( Armor& armor) const 
{
    Light l1 = armor.left_light;
    Light l2 = armor.right_light;
    // 灯条角度差
    float angle_diff = std::abs(l1.angle - l2.angle);
    if (angle_diff > param_.armor_params.max_angle_diff) {
        return false;
    }
    // 灯条长度一致性(相等)
    float length_ratio = std::min(l1.length, l2.length) / std::max(l1.length, l2.length); 
    if (length_ratio < 0.7f) {
        return false;
    }
    // 归一化中心距(用灯条自身当尺子，消除“远近”和“分辨率”的影响)
    float avg_light_length = (l1.length + l2.length) * 0.5f;
    float center_dist = cv::norm(l1.center - l2.center) / avg_light_length;

    // 判定装甲板类型
    if (center_dist >= param_.armor_params.min_small_center_dist &&
        center_dist <= param_.armor_params.max_small_center_dist) {
        armor.type = ArmorType::SMALL;
    } else if (center_dist >= param_.armor_params.min_large_center_dist &&
        center_dist <= param_.armor_params.max_large_center_dist) {
        armor.type = ArmorType::LARGE;
    } else {
        return false;
    }
    
    return true;
}

// =========== 绘制调试信息 ============== //
void Detector::drawDebug(cv::Mat& img, cv::Scalar colors) const {
    // 画灯条
    for (const auto& light : lights_) {
        cv::circle(img, light.top, 3, cv::Scalar(255, 255, 255), -1);
        cv::circle(img, light.bottom, 3, cv::Scalar(255, 255, 255), -1);
        cv::line(img, light.top, light.bottom, cv::Scalar(255, 0, 255), 2);
    }
    // 画装甲板
    for (const auto& armor : armors_) {

        const Light& left_light  = armor.left_light;
        const Light& right_light = armor.right_light;
        // 画框
        cv::line(img, left_light.top, right_light.bottom, colors, 2);
        cv::line(img, right_light.top, left_light.bottom,  colors, 2);
        cv::line(img, left_light.top, right_light.top, colors, 2);
        cv::line(img, left_light.bottom, right_light.bottom, colors, 2);

        // 画中心点
        cv::circle(img, armor.center, 4, cv::Scalar(255,0,0), -1);

        // ================= 标注装甲类型 =================
        std::string armor_text = armor.type == ArmorType::SMALL ? "SMALL" : 
                    (armor.type == ArmorType::LARGE ? "LARGE" : "UNKNOW");

        cv::Point2f text_pos = armor.center + cv::Point2f(5, -5);

        cv::putText(img, armor_text, text_pos, 
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,0,0), 2);

        // 左右灯条标记
        cv::putText(img, "L", left_light.center, cv::FONT_HERSHEY_SIMPLEX, 0.4,
                    cv::Scalar(0, 0, 255), 1);
        cv::putText(img, "R", right_light.center, cv::FONT_HERSHEY_SIMPLEX, 0.4,
                    cv::Scalar(0, 0, 255), 1);
    }
}

// =========== 读取配置参数 ============ //
bool Detector::Params::loadParams(const std::string& path) 
{
    YAML::Node root = YAML::LoadFile(path);

    // ======= pre ======== //
    // auto pre = root["preprocess"];
    binary_threshold = root["binary_threshold"].as<int>();
    detect_color = root["detect_color"].as<int>();

    // ======= Light ======== //-
    auto light = root["light"];
    light_params.min_ratio = light["min_light_ratio"].as<float>();
    light_params.max_ratio = light["max_light_ratio"].as<float>();
    light_params.max_angle = light["max_light_angle"].as<float>();

    // ======= Armor ======== //
    auto armor = root["armor"];
    armor_params.min_small_center_dist = armor["min_small_center_dist"].as<float>();
    armor_params.max_small_center_dist = armor["max_small_center_dist"].as<float>();
    armor_params.min_large_center_dist = armor["min_large_center_dist"].as<float>();
    armor_params.max_large_center_dist = armor["max_large_center_dist"].as<float>();
    armor_params.max_angle_diff        = armor["max_angle_diff"].as<float>();

    return true;
}

} // namespace auto_aim