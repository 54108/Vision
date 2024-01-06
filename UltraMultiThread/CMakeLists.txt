cmake_minimum_required(VERSION 3.12)
project(umt)

set(CMAKE_CXX_STANDARD 17)

include_directories(include)

find_package(pybind11 REQUIRED)
find_package(Python 3.10 REQUIRED COMPONENTS Development Interpreter)

include_directories(/usr/include/python3.10/)

