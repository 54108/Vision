#ifndef MV_CAMERA_HPP
#define MV_CAMERA_HPP

#include "CameraApi.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/imgproc/imgproc_c.h>
namespace mindvision
{
enum EXPOSURETIME // 相机曝光时间
{

    EXPOSURE_10000 = 10000,
    EXPOSURE_5000 = 5000,
    EXPOSURE_2500 = 2500,
    EXPOSURE_1200 = 1200,
    EXPOSURE_800 = 800,
    EXPOSURE_600 = 600,
    EXPOSURE_400 = 400,
};

enum RESOLUTION // 相机分辨率
{

    RESOLUTION_1280_X_1024,
    RESOLUTION_1280_X_800,
    RESOLUTION_800_X_600,
    RESOLUTION_640_X_480,
};

struct Camera_Resolution // 设置相机分辨率
{
    int cols;
    int rows;

    explicit Camera_Resolution(const mindvision::RESOLUTION _resolution)
    {
        switch (_resolution)
        {
        case mindvision::RESOLUTION::RESOLUTION_1280_X_1024:
            cols = 1280;
            rows = 1024;
            break;
        case mindvision::RESOLUTION::RESOLUTION_1280_X_800:
            cols = 1280;
            rows = 800;
            break;
        case mindvision::RESOLUTION::RESOLUTION_800_X_600:
            cols = 800;
            rows = 600;
            break;
        case mindvision::RESOLUTION::RESOLUTION_640_X_480:
            cols = 640;
            rows = 480;
            break;
        default:
            cols = 1280;
            rows = 800;
            break;
        }
    }
};

struct CameraParam
{
    int camera_mode;
    int camera_exposuretime;

    mindvision::Camera_Resolution resolution;

    CameraParam(const int _camera_mode, const mindvision::RESOLUTION _resolution,
                const mindvision::EXPOSURETIME _camera_exposuretime)
        : camera_mode(_camera_mode), camera_exposuretime(_camera_exposuretime), resolution(_resolution)
    {
    }
};

class MVCamera
{
  public:
    MVCamera() = default;
    explicit MVCamera(const mindvision::CameraParam &_camera_param);

    ~MVCamera();

    // 相机初始化
    int cameraInit(const int _CAMERA_RESOLUTION_COLS, const int _CAMERA_RESOLUTION_ROWS,
                   const int _CAMERA_EXPOSURETIME);

    // 相机是否在线
    bool isCameraOnline();

    // 清除缓存
    void releaseBuff();

    inline cv::Mat image() const
    {
        return cv::cvarrToMat(iplImage, true);
    }

  private:
    unsigned char *g_pRgbBuffer; // 处理后数据缓存区

    int iCameraCounts = 1;
    int iStatus = -1;
    int hCamera;
    int channel = 3;
    bool iscamera0_open = false;

    // int                     iDisplayFrames = 10000;
    tSdkCameraDevInfo tCameraEnumList;
    tSdkCameraCapbility tCapability; // 设备描述信息
    tSdkFrameHead sFrameInfo;
    tSdkImageResolution pImageResolution;
    BYTE *pbyBuffer;
    BOOL AEstate = FALSE;
    IplImage *iplImage = nullptr;
};

} // namespace mindvision

#endif