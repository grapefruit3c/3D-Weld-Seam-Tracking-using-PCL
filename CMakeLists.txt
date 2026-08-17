cmake_minimum_required(VERSION 3.20)
project(BusWelding3DVision
    VERSION 1.0.0
    DESCRIPTION "客车焊接3D视觉引导系统 - 3D Weld Seam Tracking using PCL"
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)
endif()

find_package(OpenCV REQUIRED
    COMPONENTS core calib3d imgproc imgcodecs highgui
)

find_package(PCL REQUIRED
    COMPONENTS common io filters
)

message(STATUS "========================================")
message(STATUS " 客车焊接3D视觉引导系统 - 根 CMake")
message(STATUS "========================================")
message(STATUS " OpenCV: ")
message(STATUS " PCL:    ")
message(STATUS "========================================")

add_subdirectory(modules/m1_camera_calibration)
add_subdirectory(modules/m2_depth_to_pointcloud)
