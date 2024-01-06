#include "Camera/MVCamera.hpp"
#include "Detector/ArmorDetector/ArmorDetector.hpp"
#include "Utils/fps.hpp"
#include <openvino/runtime/properties.hpp>
#include "Utils/general.hpp"


using namespace std;
using namespace cv;
using namespace mindvision;


int main()
{
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
        if (mv_capture_->isCameraOnline())
        {
            src_img_ = mv_capture_->image();
        }

        if (armor_detector.detect(src_img_, objects))
        {
            for (auto armor_object : objects)
            {
                armor_detector.display(src_img_, armor_object); // 识别结果可视化
                // poseSolver.getImgpPoints(armor_object.pts);
                // poseSolver.solvePose(apex_detector.getArmorType());
            }
        }

        imshow("output", src_img_);

        cv::waitKey(1);

        mv_capture_->releaseBuff();

        global_fps_.calculateFPSGlobal();
    }
}