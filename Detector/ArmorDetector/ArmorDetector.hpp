#ifndef YOLOXARMOR_INFERENCE_H
#define YOLOXARMOR_INFERENCE_H

#include "../../Utils/general.hpp"
#include "../OpenVINO2022/openvino_detector.hpp"
#include <Eigen/Core>
#include <ie/cpp/ie_cnn_network.h>
#include <iostream>
#include <iterator>
#include <memory>
#include <opencv2/opencv.hpp>
#include <openvino/openvino.hpp>
#include <openvino/runtime/compiled_model.hpp>
#include <openvino/runtime/tensor.hpp>
#include <string>
#include <vector>

using namespace std;
using namespace cv;
using namespace InferenceEngine;

namespace armor_detector
{

enum ArmorState
{
    LOST = 0,   // 丢失目标
    FIRST = 1,  // 第一次发现目标
    SHOOT = 2,  // 持续识别目标
    FINDING = 3 // 丢失目标但在寻找目标
};
// tips: 这里灯条四点坐标用数组和容器存储都是一样的内容，只是为了方便代码调用
struct ArmorObject
{
    Point2f apex[4];              // 灯条四点坐标（左上点起始逆时针）
    cv::Rect_<float> rect;        // 灯条四点矩形
    int cls;                      // 类别 (0:哨兵 1:英雄 2：工程 3、4、5：步兵 6：前哨站 7：基地)
    int color;                    // 颜色分类 (0:蓝色 1:红色 2:灰色)
    int area;                     // 矩形面积大小
    float prob;                   // 分类置信度
    std::vector<cv::Point2f> pts; // 灯条四点坐标（左上点起始逆时针）
    int distinguish = 0;          // 装甲板类型 (0:小装甲板 1:大装甲板)
};

class ArmorDetector
{
  public:
    ArmorDetector();
    ~ArmorDetector();
    bool detect(Mat &src, std::vector<ArmorObject> &objects);
    void display(Mat &image2show, ArmorObject object);
    bool initModel(string path);
    int getArmorType();
    int isFindTarget();

    ArmorState state = LOST;

  private:
    int isFindArmor = 0;
    ov::Core ie;
    std::shared_ptr<ov::Model> model; // 网络
    ov::CompiledModel compiled_model; // 可执行网络
    ov::InferRequest infer_request;   // 推理请求
    // ArmorState state;
    ArmorObject armor_object;
    cv::Point2f last_armor_center;

    Eigen::Matrix<float, 3, 3> transfrom_matrix;
};

} // namespace armor_detector
#endif // YOLOXARMOR_INFERENCE_H
