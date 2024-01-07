find_package(Eigen3 REQUIRED)
find_package(OpenCV 4 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

file(GLOB_RECURSE src *.cpp)

add_library(Predictor SHARED ${src})

target_link_libraries(Predictor)