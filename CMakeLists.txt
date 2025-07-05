cmake_minimum_required(VERSION 3.10)
project(OpenCVTest)

set(CMAKE_CXX_STANDARD 14)

# OpenCV 
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# Include utils directory
include_directories(${CMAKE_SOURCE_DIR}/libraries/zf_common)
include_directories(${CMAKE_SOURCE_DIR}/libraries/zf_device)
include_directories(${CMAKE_SOURCE_DIR}/libraries/zf_driver)
include_directories(${CMAKE_SOURCE_DIR}/libraries/zf_components)

include_directories(${CMAKE_SOURCE_DIR}/main/vision)
include_directories(${CMAKE_SOURCE_DIR}/main/utils/math)
include_directories(${CMAKE_SOURCE_DIR}/main/utils/time)
include_directories(${CMAKE_SOURCE_DIR}/main/utils/debug)


aux_source_directory(${CMAKE_SOURCE_DIR}/libraries/zf_common DIR_SRCS)
aux_source_directory(${CMAKE_SOURCE_DIR}/libraries/zf_device DIR_SRCS)
aux_source_directory(${CMAKE_SOURCE_DIR}/libraries/zf_driver DIR_SRCS)
aux_source_directory(${CMAKE_SOURCE_DIR}/libraries/zf_components DIR_SRCS)

aux_source_directory(main/vision/ DIR_SRCS)
aux_source_directory(main/utils/time/ DIR_SRCS)
aux_source_directory(main/utils/math/ DIR_SRCS)
aux_source_directory(main/utils/debug/ DIR_SRCS)

add_executable(OpenCVTest main/Main.cpp ${DIR_SRCS})
target_link_libraries(OpenCVTest ${OpenCV_LIBS})
