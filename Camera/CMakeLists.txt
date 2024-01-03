find_package(OpenCV 4 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

file(GLOB_RECURSE src *.cpp)

add_library(Camera OBJECT ${src})
target_link_libraries(Camera ${OpenCV_LIBS} MVSDK)
