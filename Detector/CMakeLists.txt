find_package(OpenCV 4 REQUIRED)
FIND_PACKAGE(OpenVINO REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS}/opencv2/)
include_directories(/usr/include/ie/)

list(APPEND EXTRA_INCLUDES ${PROJECT_SOURCE_DIR}/Detector/ArmorDetector ${PROJECT_SOURCE_DIR})
add_library(Detector SHARED ${PROJECT_SOURCE_DIR}/Detector/ArmorDetector/ArmorDetector.cpp)
target_link_libraries(Detector openvino::runtime ${Opencv_DIR})