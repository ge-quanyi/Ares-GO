cmake_minimum_required(VERSION 3.16)
project(wit-motion)

add_library(${PROJECT_NAME} SHARED imu_receive.cpp)

