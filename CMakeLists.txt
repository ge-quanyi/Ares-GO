cmake_minimum_required(VERSION 3.16)
project(Ares_CV)

set(CMAKE_CXX_STANDARD 14)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
SET(Eigen3_INCLUDE_DIR "/usr/include/eigen3")
find_package(OpenCV 4 REQUIRED)
find_package(fmt)
find_package(ZeroMQ REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(include)
include_directories(${Eigen3_INCLUDE_DIR})
include_directories(${ZeroMQ_INCLUDE_DIRS})

add_subdirectory(serial)
add_subdirectory(camera)
add_subdirectory(detector)
add_subdirectory(predictor)

add_executable(Ares_CV main.cpp)
target_link_libraries(Ares_CV  detector camera serial  predictor ${ZeroMQ_LIBRARY})
