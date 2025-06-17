cmake_minimum_required(VERSION 3.10)
project(OpenCVTest)

set(CMAKE_CXX_STANDARD 14)

# OpenCV 
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# Include utils directory
include_directories(${CMAKE_SOURCE_DIR}/vision)
include_directories(${CMAKE_SOURCE_DIR}/image)
include_directories(${CMAKE_SOURCE_DIR}/utils/math)
include_directories(${CMAKE_SOURCE_DIR}/utils/time)
include_directories(${CMAKE_SOURCE_DIR}/utils/debug)


aux_source_directory(vision/ DIR_SRCS)
aux_source_directory(image/ DIR_SRCS)
aux_source_directory(utils/time/ DIR_SRCS)
aux_source_directory(utils/math/ DIR_SRCS)
aux_source_directory(utils/debug/ DIR_SRCS)

add_executable(OpenCVTest Main.cpp ${DIR_SRCS})
target_link_libraries(OpenCVTest ${OpenCV_LIBS})
