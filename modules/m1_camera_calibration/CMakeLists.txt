cmake_minimum_required(VERSION 3.20)

# ==============================================================================
# M1 相机内参标定模块
# ==============================================================================

set(M1_INCLUDE_DIR "/include")

# ---- 动态库（DLL，供WPF通过P/Invoke集成） ----
add_library(M1_CalibrationLib SHARED
    src/camera_calibration.cpp
    src/camera_calibration_export.cpp
    include/camera_calibration_export.h
)

target_include_directories(M1_CalibrationLib
    PUBLIC
        $<BUILD_INTERFACE:>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        
)

target_link_libraries(M1_CalibrationLib
    PUBLIC
        
)

target_compile_definitions(M1_CalibrationLib
    PRIVATE M1_CALIBRATION_EXPORTS
)

set_target_properties(M1_CalibrationLib PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY "/lib"
    LIBRARY_OUTPUT_DIRECTORY "/bin"
    RUNTIME_OUTPUT_DIRECTORY "/bin"
    FOLDER "M1_Calibration"
    DEBUG_POSTFIX "d"
)

# ---- 可执行文件 ----
add_executable(M1_CameraCalibration
    src/main.cpp
)

target_include_directories(M1_CameraCalibration
    PRIVATE
        
        
)

target_link_libraries(M1_CameraCalibration
    PRIVATE
        M1_CalibrationLib
        
)

set_target_properties(M1_CameraCalibration PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "/bin"
    FOLDER "M1_Calibration"
    DEBUG_POSTFIX "d"
)

message(STATUS "  M1 相机标定 模块已配置")
