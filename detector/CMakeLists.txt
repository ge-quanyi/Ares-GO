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

add_library(${PROJECT_NAME} STATIC
                            src/armor_detect.cpp
                            src/ovinference.cpp)

include_directories(include)

target_link_libraries(${PROJECT_NAME}
        ${OpenCV_LIBS}
        openvino::runtime
        )