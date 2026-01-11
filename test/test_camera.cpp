#include <iostream>
#include <opencv2/opencv.hpp>
#include <io/usbcamera/usbcamera.hpp>

using namespace std;
using namespace cv;
using namespace io;

int main() {
    // ----- 初始化 ----- // 
    USBCamera camera(0);
    camera.open();

    Mat frame;
    while(1) {
        // ----- 读取图像 ----- //
        if(!camera.read(frame))
            continue;
        
        imshow("Camera_test", frame);

        // ----- 退出 ----- // 
        char key = (char)waitKey(10);
        if (key == 27)
            break;
    }
    // ----- 清除 ----- //
    camera.release();

    return 0;
}