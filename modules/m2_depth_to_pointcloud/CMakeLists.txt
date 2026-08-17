cmake_minimum_required(VERSION 3.20)

# ==============================================================================
# M2 深度图→点云生成模块
# ==============================================================================

set(M2_INCLUDE_DIR "/include")

# ---- 可执行文件 ----
add_executable(M2_DepthToPointCloud
    src/main.cpp
    src/depth_to_pointcloud.cpp
)

target_include_directories(M2_DepthToPointCloud
    PRIVATE
        
        
        
)

target_link_libraries(M2_DepthToPointCloud
    PRIVATE
        
        
)

set_target_properties(M2_DepthToPointCloud PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "/bin"
    FOLDER "M2_DepthToPointCloud"
    DEBUG_POSTFIX "d"
)

# ---- 静态库（供其他模块调用） ----
add_library(M2_DepthToPointCloudLib STATIC
    src/depth_to_pointcloud.cpp
)

target_include_directories(M2_DepthToPointCloudLib
    PUBLIC
        $<BUILD_INTERFACE:>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        
        
)

target_link_libraries(M2_DepthToPointCloudLib
    PUBLIC
        
        
)

set_target_properties(M2_DepthToPointCloudLib PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY "/lib"
    FOLDER "M2_DepthToPointCloud"
    DEBUG_POSTFIX "d"
)

message(STATUS "  M2 深度图→点云 模块已配置")
