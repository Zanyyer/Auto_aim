#ifndef IO_CAMERA_HPP
#define IO_CAMERA_HPP

#include <opencv2/opencv.hpp>
#include <string>

namespace io {

class Camera
{
public:
    Camera() = default;
    virtual ~Camera() = default;

    // 打开相机
    virtual bool open() = 0;

    // 读取一帧图像
    virtual bool read(cv::Mat& frame) = 0;

    // 是否已打开
    virtual bool isOpened() const = 0;

    // 关闭相机
    virtual void release() = 0;

    // 获取相机名称
    virtual std::string getCameraName() const = 0;
};

} // namespace io
#endif // IO_CAMERA_HPP