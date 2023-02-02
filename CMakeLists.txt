cmake_minimum_required(VERSION 3.24)
project(Ares_CV)

set(CMAKE_CXX_STANDARD 14)

find_package(OpenCV 4 REQUIRED)
find_package(fmt)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(include)
add_executable(Ares_CV main.cpp camera/src/camera.cpp camera/src/camera_api.cpp serial/serial.cpp detector/src/armor_detect.cpp detector/src/detector.cpp)
target_link_libraries(Ares_CV ${OpenCV_LIBS} fmt::fmt /usr/lib/libgxiapi.so)
