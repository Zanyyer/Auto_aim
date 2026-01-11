#include <iostream>
#include <opencv2/opencv.hpp>
#include <filesystem>

using namespace cv;
using namespace std;
namespace fs = std::filesystem;

int main() {
    Size boardSize(9, 6);

    string saveDir = "../data/calib_images";
    fs::create_directories(saveDir);

    VideoCapture cap(0);
    if (!cap.isOpened()) {
        cerr << "[ERROR] 无法打开摄像头" << endl;
        return -1;
    }

    cout << "[INFO] 按 c 保存图片,ESC 退出" << endl;

    Mat frame, gray;
    int imgCount = 0;

    while (true) {
        cap >> frame;
        if (frame.empty()) break;

        cvtColor(frame, gray, COLOR_BGR2GRAY);

        vector<Point2f> corners;
        bool found = findChessboardCorners(
            gray, boardSize, corners,
            CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE
        );

        // if (found) {
        //     drawChessboardCorners(frame, boardSize, corners, found);
        // }

        imshow("Capture", frame);
        char key = (char)waitKey(30);

        if (key == 27) break;
        else if (key == 'c' && found) {
            char filename[256];
            sprintf(filename, "%s/img_%03d.png", saveDir.c_str(), imgCount++);
            imwrite(filename, frame);
            cout << "[INFO] 保存 " << filename << endl;
        }
    }

    cap.release();
    destroyAllWindows();
    return 0;
}
