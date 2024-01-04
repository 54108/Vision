cmake_minimum_required(VERSION 3.10)
project(Vision)

set(CMAKE_CXX_STANDARD 20)

find_program(CCACHE_PROGRAM ccache)
if (CCACHE_PROGRAM)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "${CCACHE_PROGRAM}")
endif ()

add_compile_definitions(PROJECT_DIR="${PROJECT_SOURCE_DIR}")

add_compile_options(-Wall -Werror=return-type -Wno-unused-variable)
if (CMAKE_BUILD_TYPE STREQUAL Release)
    add_compile_options(-O3 -flto -march=native)
elseif (CMAKE_BUILD_TYPE STREQUAL RelWithDebInfo)
    add_compile_options(-O3 -flto -march=native -g)
elseif (CMAKE_BUILD_TYPE STREQUAL MinSizeRel)
    add_compile_options(-Os -flto -march=native)
else ()
    add_compile_options(-O0 -g)
endif ()

include_directories(/usr/include/ie/)
INCLUDE_DIRECTORIES(${InferenceEngine_INCLUDE_DIRS})

add_executable(Vision main.cpp)
target_link_libraries(Vision)

include_directories(Utils)
add_subdirectory(Utils)
target_link_libraries(Vision Utils)

include_directories(Serial)
add_subdirectory(Serial)
target_link_libraries(Vision Serial)

include_directories(Camera)
add_subdirectory(Camera)
target_link_libraries(Vision Camera)

include_directories(Detector)
add_subdirectory(Detector)
target_link_libraries(Vision Detector)

# include_directories(record)
# add_subdirectory(record)
# target_link_libraries(CVRM2021 record)
