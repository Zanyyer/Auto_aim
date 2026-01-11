#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <filesystem>

using namespace cv;
using namespace std;
namespace fs = std::filesystem;

int main() {
    Size boardSize(9, 6);
    float squareSize = 0.025f; // 米

    string imgDir  = "../data/calib_images";
    string outYaml = "../configs/camera_calib.yaml";

    vector<vector<Point2f>> imagePoints;
    vector<vector<Point3f>> objectPoints;

    // 单张棋盘的 3D 点
    vector<Point3f> obj;
    for (int i = 0; i < boardSize.height; ++i)
        for (int j = 0; j < boardSize.width; ++j)
            obj.emplace_back(j * squareSize, i * squareSize, 0);

    Size imageSize;

    // ================== 读取图片 ==================
    for (const auto& entry : fs::directory_iterator(imgDir)) {
        Mat img = imread(entry.path().string());
        if (img.empty()) continue;

        imageSize = img.size();
        Mat gray;
        cvtColor(img, gray, COLOR_BGR2GRAY);

        vector<Point2f> corners;
        bool found = findChessboardCorners(
            gray, boardSize, corners,
            CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE
        );

        if (found) {
            cornerSubPix(
                gray, corners, Size(11, 11), Size(-1, -1),
                TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.1)
            );

            imagePoints.push_back(corners);
            objectPoints.push_back(obj);

            cout << "[INFO] 检测成功: "
                 << entry.path().filename() << endl;
        }
    }

    if (imagePoints.size() < 10) {
        cerr << "[ERROR] 有效图片不足（至少 10 张）" << endl;
        return -1;
    }

    // ================== 相机标定 ==================
    Mat cameraMatrix, distCoeffs;
    vector<Mat> rvecs, tvecs;

    double rms = calibrateCamera(
        objectPoints, imagePoints, imageSize,
        cameraMatrix, distCoeffs, rvecs, tvecs
    );

    cout << "RMS 误差: " << rms << endl;

    // ================== 重投影误差分析 ==================
    double totalError = 0;
    size_t totalPoints = 0;

    for (size_t i = 0; i < objectPoints.size(); ++i) {
        vector<Point2f> projected;
        projectPoints(
            objectPoints[i], rvecs[i], tvecs[i],
            cameraMatrix, distCoeffs, projected
        );

        double err = norm(imagePoints[i], projected, NORM_L2);
        size_t n = objectPoints[i].size();

        totalError += err * err;
        totalPoints += n;

        cout << "Image " << i
             << " error: "
             << sqrt(err * err / n) << " px" << endl;
    }

    double meanError = sqrt(totalError / totalPoints);
    cout << "平均重投影误差: " << meanError << " px" << endl;

    // ================== 保存结果 ==================
    FileStorage fs(outYaml, FileStorage::WRITE);
    fs << "camera_matrix" << cameraMatrix;
    fs << "dist_coeffs" << distCoeffs;
    fs << "reprojection_error" << meanError;
    fs << "image_width" << imageSize.width;
    fs << "image_height" << imageSize.height;
    fs << "board_cols" << boardSize.width;
    fs << "board_rows" << boardSize.height;
    fs << "square_size" << squareSize;
    fs.release();

    cout << "[INFO] 标定结果已保存至 " << outYaml << endl;
    return 0;
}
