#include <iostream>
#include <opencv2/opencv.hpp>

#include "detector.hpp"

using namespace std;
using namespace cv;
using namespace auto_aim;

int main()
{
    // ========== 1. 构造 Detector 参数 ==========
    Detector::Params params;

    params.loadParams("../configs/auto_aim.yaml");

    Detector detector(params);

    // ========== 2. 打开相机 ==========
    VideoCapture cap(0, CAP_V4L2);
    if (!cap.isOpened()) {
        cerr << "[ERROR] camera open failed\n";
        return -1;
    }

    namedWindow("detector", WINDOW_AUTOSIZE);

    Mat frame;

    while (true) {
        cap >> frame;
        if (frame.empty()) {
            cerr << "[WARN] empty frame\n";
            continue;
        }

        // ========== 3. 只调用 detect ==========
        auto armors = detector.detect(frame);

        // ========== 4. 画调试 ==========
        detector.drawDebug(frame, Scalar(0,255,0));

        // 显示检测到的装甲数量
        putText(frame,
                "armors: " + to_string(armors.size()),
                Point(20, 30),
                FONT_HERSHEY_SIMPLEX,
                0.8,
                Scalar(0, 255, 0),
                2);

        imshow("detector", frame);

        char key = (char)waitKey(1);
        if (key == 27 || key == 'q') break;
    }

    cap.release();
    destroyAllWindows();
    return 0;
}
