cmake_minimum_required(VERSION 3.16)
project(camera)

find_package(OpenCV 3 REQUIRED)
find_package(fmt)

include_directories(${OpenCV_INCLUDE_DIRS})

add_library(${PROJECT_NAME} STATIC
                            src/camera_api.cpp
                            src/camera.cpp)

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} fmt::fmt /usr/lib/libgxiapi.so)