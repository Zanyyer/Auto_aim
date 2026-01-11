#include "io/usbcamera/usbcamera.hpp"
#include <iostream>

namespace io
{
    USBCamera::USBCamera(int device_id, int width, int height, int fps) 
        : device_id_(device_id), width_(width), height_(height), fps_(fps) {}

    bool USBCamera::open()
    {
        cap_.open(device_id_, cv::CAP_V4L2); // Linux 下使用 V4L2 打开摄像头
        if(!isOpened()){
            std::cerr << "[USBCamera] Failed to open device: " << device_id_ << std::endl;
            return false;
        }

        cap_.set(cv::CAP_PROP_FRAME_WIDTH, width_);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
        cap_.set(cv::CAP_PROP_FPS, fps_);

        std::cout << "[USBCamera] Opened device: " << device_id_ 
                  << " (width = " << width_ << ", height = " << height_ 
                  << ", fps = " << fps_ << ")" << std::endl;
    
        return true;
    }

    bool USBCamera::read(cv::Mat& frame)
    {
        return cap_.read(frame);
    }

    bool USBCamera::isOpened() const
    {
            return cap_.isOpened();
    }

    void USBCamera::release()
    {
        cap_.release();
        std::cout << "[USBCamera] Released device: " << device_id_ << std::endl;
    }

    void USBCamera::setConfigure(int width = -1, int height = -1, int fps = -1)
    {
        if (width > 0) width_ = width;
        if (height > 0) height_ = height;
        if (fps > 0) fps_ = fps;
    }

}