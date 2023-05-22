#CMake最小版本
cmake_minimum_required(VERSION 3.16)

#项目名称
PROJECT(606Vision VERSION 2023.3.2 LANGUAGES CXX) 

# Check system platform
# 查看系统平台
if(NOT CMAKE_SYSTEM_NAME STREQUAL Linux)
    message(FATAL_ERROR "Not supported platform: ${CMAKE_SYSTEM_NAME}")
endif()

# Specialize CMake build flags for different build types
# 
set(CMAKE_CXX_FLAGS "-Wall -Wextra -Wno-deprecated")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

# Use standard c++20
# C++20
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Add DEBUG/RELEASE marco to define if debug/release configured
#调试工具？
add_compile_options("$<$<CONFIG:DEBUG>:-DDEBUG>")
add_compile_options("$<$<CONFIG:RELEASE>:-DRELEASE>")

# Don't build test by default
# 默认不进行test？
set(BUILD_TEST False)


# 查找opencv包
find_package(OpenCV 4 REQUIRED)
find_package(OpenVINO REQUIRED COMPONENTS Runtime ONNX)
FIND_PACKAGE(Eigen3 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_libraries(${OpenCV_LIBS})

# Add fmtlib directory
add_subdirectory(3rdParty/fmt)
# Add fmtlib include to EXTRA_INCLUDES
list(APPEND EXTRA_INCLUDES 3rdParty/fmt/include)

# Add mindvision-sdk directory and include path for target
add_library(MVSDK SHARED IMPORTED)
get_filename_component(MVSDK_PATH 3rdParty/MVSDK REALPATH)
# Check system architecture for mindvision-sdk library
execute_process(COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE)
message(STATUS "Checking platform architecture for MVSDK: ${ARCHITECTURE}")
if(${ARCHITECTURE} STREQUAL x86_64)
    set(MIMDVISION_SDK_LIBRARY ${MVSDK_PATH}/lib/x64/libMVSDK.so)
elseif(${ARCHITECTURE} STREQUAL x86)
    set(MIMDVISION_SDK_LIBRARY ${MVSDK_PATH}/lib/x86/libMVSDK.so)
elseif(${ARCHITECTURE} STREQUAL aarch64)
    set(MIMDVISION_SDK_LIBRARY ${MVSDK_PATH}/lib/arm64/libMVSDK.so)
elseif(${ARCHITECTURE} STREQUAL armv7)
    set(MIMDVISION_SDK_LIBRARY ${MVSDK_PATH}/lib/arm/libMVSDK.so)
else()
    message(FATAL_ERROR "Not supported architecture for MVSDK: ${ARCHITECTURE}")
endif()
set_property(TARGET MVSDK PROPERTY IMPORTED_LOCATION ${MIMDVISION_SDK_LIBRARY})
# Add mindvision lib to build target
list(APPEND EXTRA_LIBS MVSDK)
# Add mindvision include path
list(APPEND EXTRA_INCLUDES MVSDK/include)



# Add project source dir to include
# 添加项目源代码和头文件
list(APPEND EXTRA_INCLUDES ${PROJECT_SOURCE_DIR})

# Add base src and header
# 添加可执行项目源代码和头文件
list(APPEND EXTRA_INCLUDES run_code)

list(APPEND EXTRA_INCLUDES 3rdParty/MVSDK)

# Add module as shared library to build target
# 将各个库
list(APPEND EXTRA_INCLUDES auto_aim)

list(APPEND EXTRA_INCLUDES ${PROJECT_SOURCE_DIR}/auto_aim/Detector/ArmorDetector ${PROJECT_SOURCE_DIR}/auto_aim/Detector/OpenVINO2022)
add_library(detector SHARED Detector/ArmorDetector/ArmorDetector.cpp Detector/OpenVINO2022/openvino_detector.cpp)
list(APPEND WV_EXTRA_LIBS detector)

list(APPEND EXTRA_INCLUDES ${PROJECT_SOURCE_DIR}/auto_aim/PoseSolver)
add_library(pose_solver SHARED PoseSolver/PoseSolver.cpp)
list(APPEND WV_EXTRA_LIBS pose_solver)

list(APPEND EXTRA_INCLUDES ${PROJECT_SOURCE_DIR}/auto_aim/Predictor)
add_library(predictor SHARED Predictor/PredictorKalman.cpp)
list(APPEND WV_EXTRA_LIBS predictor)

list(APPEND EXTRA_INCLUDES ${PROJECT_SOURCE_DIR}/auto_aim/Serial)
add_library(serial SHARED Serial/Serial.cpp)
list(APPEND WV_EXTRA_LIBS serial)

list(APPEND EXTRA_INCLUDES ${PROJECT_SOURCE_DIR}/auto_aim/Predictor)
add_library(kalman_filter SHARED Predictor/PredictorKalman.cpp)
list(APPEND WV_EXTRA_LIBS Predictor)

list(APPEND EXTRA_INCLUDES ${PROJECT_SOURCE_DIR}/auto_aim/MVCamera)
add_library(mindvision_camera SHARED MVCamera/MVCamera.cpp)
target_link_libraries(mindvision_camera
    PRIVATE
    MVSDK
)
list(APPEND WV_EXTRA_LIBS mindvision_camera)

# Add utils header
# list(APPEND EXTRA_INCLUDES utils)

list(APPEND EXTRA_INCLUDES ${PROJECT_SOURCE_DIR}/auto_aim/utils)
add_library(utils SHARED utils/fps.cpp utils/general.cpp)
list(APPEND WV_EXTRA_LIBS utils)



# Add EXTRA_INCLUDES to project includes
include_directories(${EXTRA_INCLUDES})
INCLUDE_DIRECTORIES(${InferenceEngine_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})

add_executable(606Vision 606/main.cpp)

# Add shared library link to 606Vision
target_link_libraries(606Vision
    PUBLIC
    fmt::fmt
    ${OpenCV_LIBS}
    ${EXTRA_LIBS}
    ${Eigen3_LIBRARIES}
    ${OpenCV_LIBS}
    openvino::frontend::onnx
    openvino::runtime
    PRIVATE
    ${WV_EXTRA_LIBS}
)

# Set WolfVision compile cxx_std features
target_compile_features(606Vision PUBLIC cxx_std_${CMAKE_CXX_STANDARD})

# Set WolfVision linking time optimization to true
set_target_properties(606Vision PROPERTIES INTERPROCEDURAL_OPTIMIZATION True)


# Set built binary to ~/bin
set(EXECUTABLE_OUTPUT_PATH "${PROJECT_BINARY_DIR}/bin")

# Set built library to ~/lib
set(LIBRARY_OUTPUT_PATH "${PROJECT_BINARY_DIR}/lib")



#TARGET_LINK_LIBRARIES(606Vision ${OpenCV_LIBS}) #链接库



