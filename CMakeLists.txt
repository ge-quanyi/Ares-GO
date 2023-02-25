cmake_minimum_required(VERSION 3.21)
project(Ares_CV)

set(CMAKE_CXX_STANDARD 14)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

find_package(OpenCV 4 REQUIRED)
find_package(fmt)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(include)

add_subdirectory(serial)
add_subdirectory(camera)
add_subdirectory(detector)
add_subdirectory(predictor)

add_executable(Ares_CV main.cpp)
target_link_libraries(Ares_CV camera serial detector)
