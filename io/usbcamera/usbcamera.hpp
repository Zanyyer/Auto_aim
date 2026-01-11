#ifndef IO__USBCAMERA__USBCAMERA_HPP
#define IO__USBCAMERA__USBCAMERA_HPP

#include "io/camera.hpp"
#include <opencv2/opencv.hpp>

namespace io
{
    class USBCamera : public Camera
    {
        public:
            explicit USBCamera(int device_id = 0, int width = 640, int height = 480, int fps = 60);
            
            bool open() override;
            bool read(cv::Mat& frame) override;
            bool isOpened() const override;
            void release() override;
            
            void setConfigure(int width, int height, int fps);
            std::string getCameraName() const override { return "USBCamera"; };

        private:
            int device_id_;
            int width_ ;
            int height_;
            int fps_;

            cv::VideoCapture cap_;
    };
}

#endif  // IO__USBCAMERA__USBCAMERA_HPP