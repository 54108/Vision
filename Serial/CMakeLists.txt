find_package(OpenCV 4 REQUIRED)
find_package(fmt REQUIRED)

file(GLOB_RECURSE src *.cpp)

include_directories( ${OpenCV_INCLUDE_DIRS} )

add_library(Serial OBJECT ${src})
target_link_libraries(Serial fmt::fmt ${Opencv_LIBS})
