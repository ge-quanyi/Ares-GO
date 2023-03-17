cmake_minimum_required(VERSION 3.16)
project(predictor)

#set(Eigen3_INCLUDES "/usr/include/eigen3")

find_package(Eigen3 REQUIRED)
find_package(PythonLibs REQUIRED)

include_directories(${Eigen3_INCLUDE_DIR})
include_directories( ${PYTHON_INCLUDE_DIRS})
add_library(${PROJECT_NAME} SHARED src/predictor.cpp include/kalmanFilter.hpp src/EKFpredictor.cpp include/AdaptiveEKF.hpp)
target_link_libraries(${PROJECT_NAME} ${Eigen3_LIBRARIES} ${PYTHON_LIBRARIES})