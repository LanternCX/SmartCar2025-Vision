cmake_minimum_required(VERSION 3.10)
project(OpenCVTest)

set(CMAKE_CXX_STANDARD 14)

# OpenCV 
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(OpenCVTest main.cpp)
target_link_libraries(OpenCVTest ${OpenCV_LIBS})
