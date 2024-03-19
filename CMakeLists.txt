cmake_minimum_required(VERSION 3.2)
project(tello_slam)

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_FLAGS "-std=c++17 -Wall")
set(CMAKE_CXX_FLAGS_RELEASE  "-std=c++17 -O3 -fopenmp -pthread")

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# DEPENDENCIES ################################################################
# CUDA
# set(CUDA_TOOLKIT_ROOT_DIR "/usr/local/cuda")
# find_package(CUDA 12 REQUIRED)

# set(CMAKE_CUDA_STANDARD 12)
# set(CMAKE_CUDA_STANDARD_REQUIRED ON)

# OpenCV
find_package(OpenCV 4 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# Eigen
include_directories("/usr/include/eigen3")

# Sophus
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

add_definitions("-D SOPHUS_USE_BASIC_LOGGING")

# pangolin
find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

# 
set(THIRD_PARTY_LIBS
    ${OpenCV_LIBS}
    ${Sophus_LIBRARIES}
    ${Pangolin_LIBRARIES})

# #############################################################################
include_directories(${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/include/camera
    ${PROJECT_SOURCE_DIR}/include/data_types
    ${PROJECT_SOURCE_DIR}/include/door
    ${PROJECT_SOURCE_DIR}/include/estimator
    ${PROJECT_SOURCE_DIR}/include/estimator/filter
    ${PROJECT_SOURCE_DIR}/include/estimator/optimizer
    ${PROJECT_SOURCE_DIR}/include/imu
    ${PROJECT_SOURCE_DIR}/include/marker
    ${PROJECT_SOURCE_DIR}/include/port
    ${PROJECT_SOURCE_DIR}/include/tool
    ${PROJECT_SOURCE_DIR}/include/toolbox
    ${PROJECT_SOURCE_DIR}/third-party)

add_subdirectory(app)
add_subdirectory(src)
add_subdirectory(test)
add_subdirectory(tool)
