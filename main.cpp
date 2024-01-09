#include "Camera/MVCamera.hpp"
#include "Detector/ArmorDetector/ArmorDetector.hpp"
#include "Detector/OpenVINO2022/types.hpp"
#include "PoseSolver/PoseSolver.hpp"
#include "Predictor/msg.hpp"
#include "Utils/fps.hpp"
#include "Utils/general.hpp"
#include <chrono>
#include <cstdlib>
#include <openvino/runtime/properties.hpp>
#include <string>

using namespace std;
using namespace cv;

msg::Armor armor_msg;
msg::Armors armors_msg;
msg::Velocity velocity;
msg::Target target;
msg::TrackerInfo tracker_info;
msg::Send send;
msg::PoseStamped posestamped;

int main()
{
    PoseSolver poseSolver = PoseSolver("Configs/pose_solver/camera_params.xml", 1);
    mindvision::MVCamera *mv_capture_ = new mindvision::MVCamera(
        mindvision::CameraParam(0, mindvision::RESOLUTION_1280_X_1024, mindvision::EXPOSURE_5000));
    cv::Mat src_img_;

    std::vector<cv::Point2f> image_points;
    armor_detector::ArmorDetector armor_detector;
    std::vector<armor_detector::ArmorObject> objects;

    // 初始化网络模型
    const string network_path = "Detector/model/opt-0517-001.xml";

    armor_detector.initModel(network_path);

    fps::FPS global_fps_;

    while (true)
    {
        global_fps_.getTick();

        std::chrono::system_clock::time_point now = std::chrono::system_clock::now();

        if (mv_capture_->isCameraOnline())
        {
            src_img_ = mv_capture_->image();
        }

        std::chrono::system_clock::time_point camera = std::chrono::system_clock::now();

        if (armor_detector.detect(src_img_, objects))
        {
            for (auto armor_object : objects)
            {
                poseSolver.solvePose(armor_object, armor_msg);
                // putText(src_img_, to_string(global_fps_.getpfs()), Point(0, 24), FONT_HERSHEY_COMPLEX, 1.0,
                //         Scalar(12, 23, 200), 3, 8);
                // putText(src_img_, to_string(poseSolver.getYawAngle()), Point(0, 48), FONT_HERSHEY_COMPLEX, 1.0,
                //         Scalar(12, 23, 200), 3, 8);
                // armor_detector.display(src_img_, armor_object); // 识别结果可视化
            }
        }

        std::chrono::system_clock::time_point solvepose = std::chrono::system_clock::now();

        cout << "cmaera:" << (camera - now).count()/1000000 << endl;
        cout << "detect and solvepose:" << (solvepose - camera).count()/1000000 << endl;

        imshow("output", src_img_);

        cv::waitKey(1);

        mv_capture_->releaseBuff();

        global_fps_.calculateFPSGlobal();
    }
}