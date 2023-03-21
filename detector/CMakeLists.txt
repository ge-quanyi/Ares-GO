cmake_minimum_required(VERSION 3.16)
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
find_package(ZeroMQ REQUIRED)
add_library(${PROJECT_NAME} STATIC
                            src/armor_detect.cpp
                            src/ovinference.cpp)

include_directories(include)
include_directories(${Eigen3_INCLUDE_DIR})
include_directories(${ZeroMQ_INCLUDE_DIRS})

target_link_libraries(${PROJECT_NAME}
        ${OpenCV_LIBS}
        openvino::runtime
        ${Eigen3_LIBRARIES}
        ${ZeroMQ_LIBRARY}
        )