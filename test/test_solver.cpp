#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>

#include "detector.hpp"
#include "solver.hpp"
#include "type.hpp"

using namespace std;
using namespace cv;
using namespace auto_aim;

int main() {
    // ========== 初始化检测器 ==========
    Detector::Params params;
    params.loadParams("../configs/auto_aim.yaml");

    Detector detector(params);
    cout << "[INFO] Detector initialized successfully!" << endl;

    // ========== 初始化解算 ==========
    string path = "../configs/auto_aim.yaml";
    Solver solver(path);

    // ========== 打开相机 ==========
    VideoCapture cap(0, CAP_V4L2);
    if (!cap.isOpened()) {
        cerr << "[ERROR] Camera open failed! Please check connection." << endl;
        return -1; 
    }

    Mat frame;
    while (true) {
        cap >> frame;
        if (frame.empty()) {
            cerr << "[WARN] Empty frame, continue..." << endl;
            waitKey(10);
            continue; 
        }

        // ========== 检测+解算 ==========
        vector<Armor> armors = detector.detect(frame);
        int solved_count = 0;

        for (auto& armor : armors) {
            if (solver.solve(armor)) {
                solved_count++;
                
                cout << "Distance to image center (px): " <<
                    solver.calculateDistanceToCenter(armor.center) << endl;
                // cout << "Solved_count : " << solved_count << endl; // test
            }
        }

        // ========== 绘制+显示 ==========
        detector.drawDebug(frame, Scalar(0,255,0));
        // solver.printSolverDebug(frame, armors);
        
        // 显示状态
        string status_text = "Detected: " + to_string(armors.size()) +
                             " | Solved: " + to_string(solved_count);
        putText(frame, status_text,
                Point(20, 20),
                FONT_HERSHEY_SIMPLEX, 0.8,
                Scalar(0, 255, 0), 2);

        imshow("Solver Test", frame);

        char key = (char)waitKey(1);
        if (key == 27 || key == 'q') {
            cout << "[INFO] User exited program." << endl;
            break;
        }
    }

    cap.release();
    destroyAllWindows();
    return 0;
}