cmake_minimum_required(VERSION 3.21)
project(serial)

find_package(fmt)

add_library(${PROJECT_NAME} STATIC serial.cpp)
target_link_libraries(${PROJECT_NAME} fmt::fmt)
