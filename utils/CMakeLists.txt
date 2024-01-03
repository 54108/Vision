find_package(OpenCV 4 REQUIRED)
find_package(fmt REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})

file(GLOB_RECURSE src *.cpp)

add_library(utils OBJECT ${src})
target_link_libraries(utils fmt::fmt ${OpenCV_LIBS})
