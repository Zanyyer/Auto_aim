// ===== c/c++  opencv ===== //
#include <iostream>
#include <csignal>
#include <chrono>
#include <opencv2/opencv.hpp>

// ===== 项目头文件 =========== //
#include "io/usbcamera/usbcamera.hpp"
#include "io/serials/serial.hpp"
#include "auto_aim/include/detector.hpp"
#include "auto_aim/include/solver.hpp"
#include "auto_aim/include/tracker.hpp"
#include "auto_aim/include/type.hpp"

// ===== 命名空间 =========== //
using namespace std;
using namespace cv;
using namespace io;
using namespace auto_aim;

static volatile sig_atomic_t g_stop_flag = 0; // 全局标志
static void signal_handler(int) { g_stop_flag = 1; } // 信号

int main(int argc, char** argv) 
{
    int cam_id = 0;
    string params_path = "../configs/auto_aim.yaml";
    string serial_port = "/dev/ttyUSB0";
    int baudrate = 115200;

    if (argc > 1) cam_id = stoi(argv[1]);
    if (argc > 2) params_path = argv[2];
    if (argc > 3) serial_port = argv[3];

    // Register signal handler for graceful exit
    signal(SIGINT, signal_handler);

    // ================ 1. 初始化 ================ //
    
    // ----- 1.1 USBCamera ----- //
    USBCamera camera(cam_id);
    if(!camera.open()){
        cerr << "[ERROR] Camera open failed (id=" << cam_id << ")" << endl;
        return -1;
    }
    cout << "[INFO] Camera initialized !" << endl;
    
    // ----- 1.2 Detector ----- //
    Detector::Params params;
    if(!params.loadParams(params_path)) {
        cerr << "[ERROR] Failed to load detector params: " << params_path << endl;
        return -1;
    }
    Detector detector(params);
    cout << "[INFO] Detector initialized !" << endl;

    // ----- 1.3 Solver ----- //
    Solver solver(params_path);
    cout << "[INFO] Solver initialized." << endl;

    // ----- 1.4 Tracker ----- //
    ArmorTracker tracker;
    cout << "[INFO] Tracker initialized." << endl;

    // ----- 1.5 Serials ----- //
    Serial serial(serial_port, baudrate);
    if (serial.open()) {
        cout << "[INFO] Serial port " << serial_port << " opened successfully!" << endl;
    } else {
        cerr << "[WARN] Serial port open failed, continuing without serial output" << endl;
    }

    // =============== 2. 主程序 =============== //
    Mat frame;
    namedWindow("RESULT", WINDOW_AUTOSIZE);
    // resizeWindow("RESULT", 640, 480);
    while (!g_stop_flag) {
        if(!camera.read(frame)){
            cerr << "[WARN] Empty frame, continue..." << endl;
            if (g_stop_flag) break;
            continue;
        }

        // ----- 2.1 检测装甲板 ----- //
        vector<Armor> armors = detector.detect(frame);
        
        // ----- 2.2 位姿解算 ----- //
        int solved_count = 0;
        Armor* best_armor = nullptr;
        for(Armor& armor : armors)  { 
            if(!solver.solve(armor)) {continue;}
            ++solved_count;
            
            if(!best_armor || best_armor->position.y > armor.position.y){
                best_armor = &armor;
            } 
        }

        // ----- 2.3 卡尔曼预测 ----- //
        if(best_armor != nullptr) {
            auto current_time = std::chrono::steady_clock::now();
            tracker.update(best_armor->position, current_time);
        }
        double dt = 0.03; // 30 ms
        tracker.predict(dt); // 预测

        if (best_armor != nullptr) {
            Point3f pred_pos = tracker.getPrePosition();
            
            Vec3d pred_tvec(static_cast<double>(pred_pos.x),
                            static_cast<double>(pred_pos.y),
                            static_cast<double>(pred_pos.z));
            vector<Point2f> pred_img_pts;

            solver.projectArmorPoints(*best_armor, best_armor->rvec, pred_tvec, pred_img_pts);
            if (pred_img_pts.size() == 4) {
                for (int i = 0; i < 4; ++i) {
                    line(frame, pred_img_pts[i], pred_img_pts[(i+1)%4], Scalar(0,255,255), 2);
                    circle(frame, pred_img_pts[i], 3, Scalar(0,255,255), -1);
                }
            }
        }

        // --- 2.4 显示结果 --- // 
        detector.drawDebug(frame, Scalar(0,255,0));
        tracker.drawPredict(frame, 
                            solver.getCameraMatrix(), 
                            solver.getDistCoeffs(), 
                            Scalar(0,0,255)
                           );
        
        string text = "Detected: " + to_string(armors.size()) +
                " | Solved: " + to_string(solved_count);
        putText(frame, text, Point(20, 20), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 255, 255), 2);
        imshow("RESULT", frame);

        // --- 2.5 串口发送数据 --- //
        if (serial.isOpened() && best_armor != nullptr) {
            // 打包数据为字符串，确保拼接正确
            string send_str = string("SOLVED:") + to_string(best_armor->position.x) + "," +
                              to_string(best_armor->position.y) + "," +
                              to_string(best_armor->position.z) + "\n" +
                              string("PRED:") + to_string(tracker.getPrePosition().x) + "," +
                              to_string(tracker.getPrePosition().y) + "," +
                              to_string(tracker.getPrePosition().z) + "\n";

            const uint8_t* data = reinterpret_cast<const uint8_t*>(send_str.c_str());
            serial.send(data, send_str.size());
        }

        // --- 2.6 退出 --- //
        char key = (char)waitKey(30);
        if (key == 27 || key == 'q') {
            cout << "[INFO] User exited program." << endl;
            break;
        }
    }

    // ============ 3. 释放 ============= //
    if (serial.isOpened()) serial.close();
    if (camera.isOpened()) camera.release();
    destroyAllWindows();

    cout << "[INFO] Program terminated." << endl;
    return 0;
}