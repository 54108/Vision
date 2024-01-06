find_package(OpenCV 4 REQUIRED)
find_package(fmt REQUIRED)

file(GLOB_RECURSE src *.cpp)

include_directories( ${OpenCV_INCLUDE_DIRS} )
list(APPEND EXTRA_INCLUDES ${PROJECT_SOURCE_DIR}/PoseSolver)

add_library(PoseSolver SHARED PoseSolver.cpp)
target_link_libraries(PoseSolver ${Opencv_LIBS})