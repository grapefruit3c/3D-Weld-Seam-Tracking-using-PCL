#ifndef DEPTH_TO_POINTCLOUD_HPP
#define DEPTH_TO_POINTCLOUD_HPP

/**
 * @file depth_to_pointcloud.hpp
 * @brief M2 深度图→点云生成模块
 * @description
 *   将深度图（Depth Map）转换为 3D 点云，是相机标定后的核心步骤。
 *   输入：深度图（16位PNG或32位浮点TIFF）+ 相机内参（来自M1标定结果）
 *   输出：PCL PointCloud（XYZ或XYZRGB格式）
 *
 * 原理：
 *   对于深度图上的每个像素 (u, v)，已知深度值 d，相机内参 fx, fy, cx, cy：
 *     Z = d / depth_scale
 *     X = (u - cx) * Z / fx
 *     Y = (v - cy) * Z / fy
 *
 * 如果你已经在Python中跑通了深度图→点云，本模块的C++实现与之逻辑一致，
 * 但做了以下优化：
 *   1. 使用OpenCV矩阵运算加速像素坐标转换
 *   2. 支持组织化点云（保留图像结构）和非组织化点云（去噪后）
 *   3. 多线程并行处理（OpenCV自带）
 *   4. 深度图可视化（伪彩色渲染）
 *
 * 数据流：
 *   M1标定结果 → 读取深度图 → 像素坐标到3D坐标转换 → 点云滤波 → 保存PCD/PLY
 *
 * 依赖：
 *   - OpenCV (core, imgproc, imgcodecs)
 *   - PCL (common, io)
 */

#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/filter.h>

#include <string>
#include <memory>
#include <vector>

namespace bus_welding {

/**
 * @brief 深度图→点云转换器
 *
 * 核心功能：
 * 1. 深度图（CV_16UC1/CV_32FC1）→ 3D点云（XYZ/XYZRGB）
 * 2. 支持组织化（organized）和非组织化点云
 * 3. 深度值滤波（去除无效点）
 * 4. 点云保存（PCD/PLY格式）
 * 5. 深度图可视化
 *
 * 使用示例：
 * @code
 *   // 1. 加载深度图（从相机或文件）
 *   cv::Mat depth = cv::imread("depth.png", cv::IMREAD_UNCHANGED);
 *
 *   // 2. 加载相机内参（来自M1标定结果）
 *   cv::Mat K = (cv::Mat_<double>(3,3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
 *
 *   // 3. 转换
 *   DepthToPointCloud converter;
 *   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
 *   converter.depthToPointCloud(depth, K, cloud);
 *
 *   // 4. 保存
 *   converter.savePointCloud("output.pcd", cloud);
 * @endcode
 */
class DepthToPointCloud {
public:
    /// 深度图像素格式
    enum class DepthFormat {
        CV_16UC1_MM,      ///< 16位无符号整数，单位mm（最常见的结构光/双目深度图）
        CV_16UC1_CM,      ///< 16位无符号整数，单位cm
        CV_32FC1_M,       ///< 32位浮点，单位m（ToF相机常用）
        CV_32FC1_MM,      ///< 32位浮点，单位mm
        CV_16UC1_RAW,     ///< 16位原始值，需自定义depth_scale
        CV_32FC1_RAW      ///< 32位原始值，需自定义depth_scale
    };

    /// 点云输出格式
    enum class OutputFormat {
        PCD_BINARY,       ///< PCD 二进制格式（推荐，体积小，速度快）
        PCD_ASCII,        ///< PCD ASCII格式（可读，但文件大）
        PLY_BINARY,       ///< PLY 二进制格式（通用性更好）
        PLY_ASCII         ///< PLY ASCII格式
    };

    /// 配置参数
    struct Config {
        // ---- 深度值参数 ----
        DepthFormat depth_format = DepthFormat::CV_16UC1_MM;  ///< 深度图格式
        float depth_scale = 1.0f;      ///< 深度缩放因子（仅在RAW模式下使用）
        float max_depth_mm = 5000.0f;   ///< 最大有效深度 (mm)，超过此值视为无效点
        float min_depth_mm = 0.0f;      ///< 最小有效深度 (mm)，低于此值视为无效点

        // ---- 点云选项 ----
        bool organized = true;           ///< 是否输出组织化点云（保留图像行列结构）
        bool remove_nan = true;          ///< 移除 NaN 点
        bool remove_inf = true;          ///< 移除 Inf 点
        bool remove_zero_depth = true;   ///< 移除深度为0的点
        int step_rows = 1;               ///< 行采样步长（1=全部，2=隔行采样）
        int step_cols = 1;               ///< 列采样步长

        // ---- 输出选项 ----
        OutputFormat output_format = OutputFormat::PCD_BINARY;  ///< 输出格式
        bool save_organized = false;     ///< 组织化点云是否保存行列索引
    };

    DepthToPointCloud();
    explicit DepthToPointCloud(const Config& config);
    ~DepthToPointCloud() = default;

    // 禁止拷贝
    DepthToPointCloud(const DepthToPointCloud&) = delete;
    DepthToPointCloud& operator=(const DepthToPointCloud&) = delete;

    // ========================================================================
    // 配置
    // ========================================================================
    const Config& getConfig() const { return config_; }
    void setConfig(const Config& config) { config_ = config; }

    // ========================================================================
    // 核心转换方法
    // ========================================================================

    /**
     * @brief 深度图 → 3D点云 (XYZ)
     * @param depth_image  深度图（CV_16UC1 或 CV_32FC1）
     * @param camera_matrix 相机内参矩阵 (3x3, 来自M1标定结果)
     * @param cloud        输出点云
     * @return true 成功
     */
    bool depthToPointCloud(
        const cv::Mat& depth_image,
        const cv::Mat& camera_matrix,
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    /**
     * @brief 深度图 + 彩色图 → 彩色点云 (XYZRGB)
     * @param depth_image  深度图
     * @param color_image  彩色图（BGR格式，与深度图对齐）
     * @param camera_matrix 相机内参矩阵
     * @param cloud        输出彩色点云
     * @return true 成功
     */
    bool depthToColoredPointCloud(
        const cv::Mat& depth_image,
        const cv::Mat& color_image,
        const cv::Mat& camera_matrix,
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

    // ========================================================================
    // 点云IO
    // ========================================================================

    /// 保存点云到文件
    bool savePointCloud(
        const std::string& file_path,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    /// 保存彩色点云
    bool saveColoredPointCloud(
        const std::string& file_path,
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

    /// 加载点云
    bool loadPointCloud(
        const std::string& file_path,
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    // ========================================================================
    // 工具方法
    // ========================================================================

    /// 深度图渲染为伪彩色图（用于可视化）
    cv::Mat renderDepthAsColor(const cv::Mat& depth_image);

    /// 深度图渲染为热力图（带颜色条）
    cv::Mat renderDepthColorMap(const cv::Mat& depth_image);

    /// 获取点云统计信息
    struct CloudStats {
        int width = 0;
        int height = 0;
        size_t total_points = 0;
        size_t valid_points = 0;
        size_t invalid_points = 0;
        float min_depth = 0.0f;
        float max_depth = 0.0f;
        float mean_depth = 0.0f;
        float x_range = 0.0f;
        float y_range = 0.0f;
        float z_range = 0.0f;
    };
    CloudStats computeCloudStats(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    /// 生成点云统计报告
    std::string generateStatsReport(const CloudStats& stats);

    /// 获取版本号
    static std::string getVersion() { return "M2_DepthToPointCloud_v1.0.0"; }

private:
    Config config_;

    /// 根据配置获取深度缩放因子
    float getDepthScale() const;

    /// 检查深度值是否有效
    bool isDepthValid(float depth_mm) const;

    /// 内部：组织化深度图 → 组织化点云
    bool organizedDepthToCloud(
        const cv::Mat& depth_image,
        const cv::Mat& camera_matrix,
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    /// 内部：非组织化深度图 → 非组织化点云（自动去噪）
    bool unorganizedDepthToCloud(
        const cv::Mat& depth_image,
        const cv::Mat& camera_matrix,
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    /// 内部：深度值到mm的转换
    float depthToMillimeters(float depth_value) const;
};

} // namespace bus_welding

#endif // DEPTH_TO_POINTCLOUD_HPP
