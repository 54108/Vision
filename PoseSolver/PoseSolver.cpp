#include "PoseSolver.hpp"
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

PoseSolver::PoseSolver(const char *filePath, int camId)
{
    FileStorage fsRead;
    fsRead.open(filePath, FileStorage::READ);
    if (!fsRead.isOpened())
    {
        cout << "Failed to open xml" << endl;
    }

    // fsRead["Y_DISTANCE_BETWEEN_GUN_AND_CAM"] >> GUN_CAM_DISTANCE_Y;

    Mat cameraMatrix;
    // Mat distortionCoeffs;
    switch (camId)
    {
    case 1:
        fsRead["CAMERA_MATRIX_1"] >> cameraMatrix;
        fsRead["DISTORTION_COEFF_1"] >> distortionCoeffs;
        break;
    default:
        cout << "WRONG CAMID GIVEN!" << endl;
        break;
    }
    setCameraParams(cameraMatrix, distortionCoeffs);
    fsRead.release();
    // Unit: m
    constexpr double small_half_width = SMALL_ARMOR_WIDTH / 2.0 / 1000.0;
    constexpr double small_half_height = SMALL_ARMOR_HEIGHT / 2.0 / 1000.0;
    constexpr double large_half_ywidth = LARGE_ARMOR_WIDTH / 2.0 / 1000.0;
    constexpr double large_half_height = LARGE_ARMOR_HEIGHT / 2.0 / 1000.0;

    // Start from bottom left in clockwise order
    // Model coordinate: x forward, y left, z up
    smallObjPoints.emplace_back(Point3f(-small_half_width, small_half_height, 0));  // tl top left左上
    smallObjPoints.emplace_back(Point3f(-small_half_width, -small_half_height, 0)); // bl below left左下
    smallObjPoints.emplace_back(Point3f(small_half_width, -small_half_height, 0));  // br below right右下
    smallObjPoints.emplace_back(Point3f(small_half_width, small_half_height, 0));   // tr top right右上

    bigObjPoints.emplace_back(Point3f(-large_half_ywidth, large_half_height, 0));  // tl top left左上
    bigObjPoints.emplace_back(Point3f(-large_half_ywidth, -large_half_height, 0)); // br below right左下
    bigObjPoints.emplace_back(Point3f(large_half_ywidth, -large_half_height, 0));  // bl below left右下
    bigObjPoints.emplace_back(Point3f(large_half_ywidth, large_half_height, 0));   // tr top right右上
}

PoseSolver::~PoseSolver(void)
{
}

void PoseSolver::setCameraParams(const Mat &camMatrix, const Mat &distCoeffs)
{
    instantMatrix = camMatrix;
    distortionCoeffs = distCoeffs;
}

int PoseSolver::readFile(const char *filePath, int camId)
{
    FileStorage fsRead;
    fsRead.open(filePath, FileStorage::READ);
    if (!fsRead.isOpened())
    {
        cout << "Failed to open xml" << endl;
        return -1;
    }

    // fsRead["Y_DISTANCE_BETWEEN_GUN_AND_CAM"] >> GUN_CAM_DISTANCE_Y;

    Mat cameraMatrix;
    // Mat distortionCoeffs;
    switch (camId)
    {
    case 1:
        fsRead["CAMERA_MATRIX_1"] >> cameraMatrix;
        fsRead["DISTORTION_COEFF_1"] >> distortionCoeffs;
        break;
    default:
        cout << "WRONG CAMID GIVEN!" << endl;
        break;
    }
    setCameraParams(cameraMatrix, distortionCoeffs);
    fsRead.release();
    return 0;
}

// void PoseSolver::setObjPoints(int type)
// {
//     double large_half_ywidth, large_half_height;
//     switch (type)
//     {
//     case smallArmor:
//         large_half_ywidth = SMALL_ARMOR_WIDTH / 2.0;
//         large_half_height = SMALL_ARMOR_HEIGHT / 2.0;
    //     smallObjPoints.emplace_back(Point3f(-large_half_ywidth, large_half_height, 0));  // tl top left左上
    //     smallObjPoints.emplace_back(Point3f(-large_half_ywidth, -large_half_height, 0)); // bl below left左下
    //     smallObjPoints.emplace_back(Point3f(large_half_ywidth, -large_half_height, 0));  // br below right右下
    //     smallObjPoints.emplace_back(Point3f(large_half_ywidth, large_half_height, 0));   // tr top right右上
    //     break;

    // case bigArmor:
    //     large_half_ywidth = LARGE_ARMOR_WIDTH / 2.0;
    //     large_half_height = LARGE_ARMOR_HEIGHT / 2.0;
    //     bigObjPoints.emplace_back(Point3f(-large_half_ywidth, large_half_height, 0));  // tl top left左上
    //     bigObjPoints.emplace_back(Point3f(-large_half_ywidth, -large_half_height, 0)); // br below right左下
    //     bigObjPoints.emplace_back(Point3f(large_half_ywidth, -large_half_height, 0));  // bl below left右下
    //     bigObjPoints.emplace_back(Point3f(large_half_ywidth, large_half_height, 0));   // tr top right右上
//         break;
//     default:
//         break;
//     }
// }

void PoseSolver::getImgpPoints(std::vector<Point2f> image_points)
{
    imagePoints.clear();
    for (int i = 0; i < 4; i++)
    {
        imagePoints.emplace_back(image_points[i]);
        // cout << endl << imagePoints[i] << endl;
    }
    cout << endl;
}

void PoseSolver::solvePose(int armorType)
{
    if (imagePoints.size() == 0)
    {
        cout << "未获取到图像点" << endl;
        return;
    }
    tvec = cv::Mat::zeros(3, 1, CV_64FC1);
    switch (armorType)
    {
    case smallArmor:
        solvePnP(smallObjPoints, imagePoints, instantMatrix, distortionCoeffs, rvec, tvec, false, SOLVEPNP_ITERATIVE);
        cout << "\n小装甲板:" << endl;
        break;
    case bigArmor:
        solvePnP(bigObjPoints, imagePoints, instantMatrix, distortionCoeffs, rvec, tvec, false, SOLVEPNP_ITERATIVE);
        cout << "\n大装甲板:" << endl;
        break;
    default:
        break;
    }

    // rotT.at<double>(1, 0) -= GUN_CAM_DISTANCE_Y;
    double x_pos = tvec.at<double>(0, 0);
    double y_pos = tvec.at<double>(1, 0);
    double z_pos = tvec.at<double>(2, 0);
    cout << "==================================\n" << tvec << endl;
    // cout << "---------------------------------\n" << x_pos << endl;

    double tan_pitch = y_pos / sqrt(x_pos * x_pos + z_pos * z_pos);
    double tan_yaw = x_pos / z_pos;

    pnp_results.yaw_angle = static_cast<float>(atan(tan_yaw) * 180 / CV_PI);
    pnp_results.pitch_angle = static_cast<float>(-atan(tan_pitch) * 180 / CV_PI);
    pnp_results.distance = static_cast<float>(sqrt(x_pos * x_pos + y_pos * y_pos + z_pos * z_pos));
}

float PoseSolver::getYawAngle()
{
    return pnp_results.yaw_angle;
}

float PoseSolver::getPitchAngle()
{
    return pnp_results.pitch_angle;
}

float PoseSolver::getDistance()
{
    return pnp_results.distance;
}
