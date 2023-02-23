cmake_minimum_required(VERSION 3.21)
project(predictor)

find_package(Eigen3 REQUIRED)
find_package(PythonLibs REQUIRED)
include_directories(${Eigen3_INCLUDES})
include_directories( ${PYTHON_INCLUDE_DIRS})
add_library(${PROJECT_NAME} STATIC src/predictor.cpp include/kalmanFilter.hpp)
target_link_libraries(${PROJECT_NAME} ${Eigen3_LIBRARIES} ${PYTHON_LIBRARIES})