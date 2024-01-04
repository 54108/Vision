find_package(OpenCV 4 REQUIRED)
FIND_PACKAGE(OpenVINO REQUIRED)
find_package(Eigen3 REQUIRED)

set(OpenVINO_INCLUDE_DIRS "/usr/include/ie/")

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${OpenVINO_INCLUDE_DIRS})

link_directories(/usr/lib)

list(APPEND EXTRA_INCLUDES ${PROJECT_SOURCE_DIR}/Detector/ArmorDetector)
add_library(Detector SHARED ${PROJECT_SOURCE_DIR}/Detector/ArmorDetector/ArmorDetector.cpp)
target_link_libraries(Detector ${Opencv_DIR})