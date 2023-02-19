cmake_minimum_required(VERSION 3.21)
project(detector)

include_directories(
        #OpenVINO推理引擎的头文件
        /opt/intel/openvino2022/runtime/include/
)

#查找必要的依赖包
find_package(OpenCV REQUIRED)
set(InferenceEngine_DIR "/opt/intel/openvino2022/runtime/cmake")
find_package(InferenceEngine)
set(ngraph_DIR "/opt/intel/openvino2022/runtime/cmake")
find_package(ngraph REQUIRED)
find_package(Eigen3 REQUIRED)
add_library(${PROJECT_NAME} STATIC
                            src/armor_detect.cpp
                            src/ovinference.cpp
                            include/anglesolver.hpp)

include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIR})
target_link_libraries(${PROJECT_NAME}
        ${OpenCV_LIBS}
        openvino::runtime
        ${Eigen3_LIBRARIES}
        )