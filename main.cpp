#include "Camera/MVCamera.hpp"
#include "utils/fps.hpp"

using namespace std;
using namespace cv;
using namespace mindvision;

int main()
{

    mindvision::MVCamera *mv_capture_ = new mindvision::MVCamera(
        mindvision::CameraParam(0, mindvision::RESOLUTION_1280_X_1024, mindvision::EXPOSURE_5000));
    cv::Mat src_img_;

    fps::FPS global_fps_;

    while (true)
    {
        global_fps_.getTick();
        if (mv_capture_->isCameraOnline())
        {
            src_img_ = mv_capture_->image();
        }

        imshow("output", src_img_);

        cv::waitKey(1);

        mv_capture_->releaseBuff();

        global_fps_.calculateFPSGlobal();
    }
}