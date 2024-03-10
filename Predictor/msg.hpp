#ifndef MSG_
#define MSG_
#include <Eigen/Eigen>
#include <Eigen/src/Geometry/Quaternion.h>
#include <chrono>
#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
namespace msg
{

typedef struct Velocity
{
    float velocity;
} Velocity;

typedef struct Target
{
    std::chrono::time_point<std::chrono::system_clock> timestamp;
    std::string id;
    float yaw;
    float v_yaw;
    int armors_num;
    float radius_1;
    float radius_2;
    cv::Point3d position;
    float dz;
    cv::Point3d velocity;
    bool tracking;
} Target;

typedef struct Armor
{
  public:
    std::string number;
    std::string type;
    float distance_to_image_center;
    struct pose
    {
        cv::Point3d position;
        Eigen::Quaterniond orientation;
    } pose;
} Armor;

typedef struct Armors
{
    std::chrono::time_point<std::chrono::system_clock> timestamp;
    std::vector<Armor> armors;
} Armors;

typedef struct TrackerInfo
{
    double position_diff;
    double yaw_diff;
    cv::Point3d position;
    double yaw;
} TrackerInfo;

typedef struct Send
{
    Target target;
    Armors armors;
    TrackerInfo tracker_info;
    cv::Point3f position;
    bool tracking;
    float pitch;
    float yaw;
} Send;

typedef struct PoseStamped
{
    struct pose
    {
        cv::Point3d position;
        Eigen::Quaterniond orientation;
    } pose;
} PoseStamped;

} // namespace msg
#endif