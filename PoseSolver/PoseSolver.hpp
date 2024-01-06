#ifndef POSE_HPP
#define POSE_HPP
#include "opencv2/core/core.hpp"
#include <iostream>
#include <opencv2/opencv.hpp>

enum ArmorType
{
    smallArmor = 0,
    bigArmor = 1
};

struct PnP_Results
{
    float yaw_angle;
    float pitch_angle;
    int distance;
    PnP_Results()
    {
        yaw_angle = 0.f;
        pitch_angle = 0.f;
        distance = 0;
    }
};

struct PnPConfig
{
    PnPConfig(const PnPConfig &) = default;
    PnPConfig(PnPConfig &&) = default;
    PnPConfig &operator=(const PnPConfig &) = default;
    PnPConfig &operator=(PnPConfig &&) = default;
    int smallArmorHeight = 60;
    int smallArmorWidth = 140;

    int bigArmorHeight = 60;
    int bigArmorWidth = 245;
};

class PoseSolver
{
  public:
    // CalculateResults calResults;

    // CalculateResults* calResult =
    // (CalculateResults*)malloc(sizeof(calResults));

    PoseSolver() = default;
    explicit PoseSolver(const char *filePath, int camId);

    ~PoseSolver();

    // CalculateResults* poseSolve();

    void setCameraParams(const cv::Mat &camMatrix, const cv::Mat &distCoeffs);
    int readFile(const char *filePath, int camId);

    // void setObjPoints(ArmorType type, double width, double height);

    void getImgpPoints(std::vector<cv::Point2f> image_points);
    void solvePose(int armorType);

    float getYawAngle();

    float getPitchAngle();

    int getDistance();

    std::tuple<double, double, double> getPose();

    void runPoseSolver();

  private:
    PnP_Results pnp_results;

    cv::Mat instantMatrix;    // Camera Matrix
    cv::Mat distortionCoeffs; // Distortion Coeffs of Camera

    std::vector<cv::Point3f> bigObjPoints;
    std::vector<cv::Point3f> smallObjPoints;

    std::vector<cv::Point2f> imagePoints;

    static constexpr float SMALL_ARMOR_WIDTH = 135;
    static constexpr float SMALL_ARMOR_HEIGHT = 55;
    static constexpr float LARGE_ARMOR_WIDTH = 225;
    static constexpr float LARGE_ARMOR_HEIGHT = 55;

    // cv::Mat rvec ;
    cv::Mat tvec ;

    cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1); //

    double pitch, yaw, distance;

    ArmorType armorType;

    double GUN_CAM_DISTANCE_Y;
};

#endif