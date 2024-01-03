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

include_directories(UltraMultiThread/include)

add_executable(Vision main.cpp)
target_link_libraries(Vision)

include_directories(utils)
add_subdirectory(utils)
target_link_libraries(Vision utils)

include_directories(Serial)
add_subdirectory(Serial)
target_link_libraries(Vision Serial)

include_directories(Camera)
add_subdirectory(Camera)
target_link_libraries(Vision Camera)

# include_directories(autoaim)
# add_subdirectory(autoaim)
# target_link_libraries(CVRM2021 autoaim)

# include_directories(record)
# add_subdirectory(record)
# target_link_libraries(CVRM2021 record)
