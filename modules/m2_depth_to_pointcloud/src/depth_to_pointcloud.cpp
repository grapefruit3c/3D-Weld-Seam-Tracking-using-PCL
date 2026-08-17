/**
 * @file depth_to_pointcloud.cpp
 * @brief M2 深度图→点云生成模块 - 实现
 * @description
 *   实现深度图到3D点云的转换，核心是相机投影模型的反向计算：
 *     Z = depth / scale
 *     X = (u - cx) * Z / fx
 *     Y = (v - cy) * Z / fy
 *
 * 支持多种深度图格式（16位/32位，mm/cm/m），
 * 输出组织化或非组织化点云，支持PCD和PLY格式。
 */

#include "depth_to_pointcloud.hpp"
#include <iostream>
#include <algorithm>
#include <limits>
#include <sstream>
#include <iomanip>

namespace bus_welding {

// ============================================================================
// 构造函数
// ============================================================================

DepthToPointCloud::DepthToPointCloud()
    : config_(Config()) {
}

DepthToPointCloud::DepthToPointCloud(const Config& config)
    : config_(config) {
}

// ============================================================================
// 内部工具方法
// ============================================================================

float DepthToPointCloud::getDepthScale() const {
    switch (config_.depth_format) {
        case DepthFormat::CV_16UC1_MM:   return 1.0f;      // 1 count = 1 mm
        case DepthFormat::CV_16UC1_CM:   return 10.0f;     // 1 count = 1 cm = 10 mm
        case DepthFormat::CV_32FC1_M:    return 1000.0f;   // 1.0 = 1 m = 1000 mm
        case DepthFormat::CV_32FC1_MM:   return 1.0f;      // 1.0 = 1 mm
        case DepthFormat::CV_16UC1_RAW:  return config_.depth_scale;
        case DepthFormat::CV_32FC1_RAW:  return config_.depth_scale;
        default:                         return 1.0f;
    }
}

float DepthToPointCloud::depthToMillimeters(float depth_value) const {
    return depth_value * getDepthScale();
}

bool DepthToPointCloud::isDepthValid(float depth_mm) const {
    if (config_.remove_zero_depth && depth_mm < 0.001f) return false;
    if (config_.remove_nan && std::isnan(depth_mm)) return false;
    if (config_.remove_inf && std::isinf(depth_mm)) return false;
    if (depth_mm < config_.min_depth_mm) return false;
    if (depth_mm > config_.max_depth_mm) return false;
    return true;
}

// ============================================================================
// 核心转换：深度图 → 组织化点云
// ============================================================================

bool DepthToPointCloud::organizedDepthToCloud(
    const cv::Mat& depth_image,
    const cv::Mat& camera_matrix,
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    if (depth_image.empty() || camera_matrix.empty()) {
        std::cerr << "[ERROR] 深度图或内参矩阵为空" << std::endl;
        return false;
    }

    int height = depth_image.rows;
    int width = depth_image.cols;

    // 提取相机内参
    double fx = camera_matrix.at<double>(0, 0);
    double fy = camera_matrix.at<double>(1, 1);
    double cx = camera_matrix.at<double>(0, 2);
    double cy = camera_matrix.at<double>(1, 2);

    // 准备组织化点云
    cloud->width = width;
    cloud->height = height;
    cloud->is_dense = false;
    cloud->points.resize(width * height);

    // 转换深度图类型到32位浮点（统一处理）
    cv::Mat depth_32f;
    if (depth_image.type() == CV_16UC1) {
        depth_image.convertTo(depth_32f, CV_32F);
    } else if (depth_image.type() == CV_32FC1) {
        depth_32f = depth_image.clone();
    } else {
        std::cerr << "[ERROR] 不支持的深度图类型: " << depth_image.type() << std::endl;
        return false;
    }

    // 主循环：逐像素转换
    // 使用OpenCV的并行处理（cv::parallel_for_）可进一步加速
    #pragma omp parallel for collapse(2) if(width * height > 100000)
    for (int v = 0; v < height; v += config_.step_rows) {
        for (int u = 0; u < width; u += config_.step_cols) {
            float depth_val = depth_32f.at<float>(v, u);
            float depth_mm = depthToMillimeters(depth_val);

            int idx = v * width + u;
            pcl::PointXYZ& pt = cloud->points[idx];

            if (isDepthValid(depth_mm)) {
                // 反向投影：像素坐标 → 相机坐标系下的3D坐标
                pt.z = depth_mm;
                pt.x = static_cast<float>((u - cx) * pt.z / fx);
                pt.y = static_cast<float>((v - cy) * pt.z / fy);
            } else {
                // 无效点设为 NaN（PCL中用NaN表示无效点）
                pt.x = pt.y = pt.z = std::numeric_limits<float>::quiet_NaN();
            }
        }
    }

    return true;
}

// ============================================================================
// 核心转换：深度图 → 非组织化点云（去噪）
// ============================================================================

bool DepthToPointCloud::unorganizedDepthToCloud(
    const cv::Mat& depth_image,
    const cv::Mat& camera_matrix,
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    if (depth_image.empty() || camera_matrix.empty()) {
        return false;
    }

    int height = depth_image.rows;
    int width = depth_image.cols;

    double fx = camera_matrix.at<double>(0, 0);
    double fy = camera_matrix.at<double>(1, 1);
    double cx = camera_matrix.at<double>(0, 2);
    double cy = camera_matrix.at<double>(1, 2);

    // 转换深度图为32位浮点
    cv::Mat depth_32f;
    if (depth_image.type() == CV_16UC1) {
        depth_image.convertTo(depth_32f, CV_32F);
    } else if (depth_image.type() == CV_32FC1) {
        depth_32f = depth_image.clone();
    } else {
        return false;
    }

    // 第一步：收集所有有效点
    std::vector<pcl::PointXYZ> valid_points;
    valid_points.reserve(width * height);  // 预分配

    for (int v = 0; v < height; v += config_.step_rows) {
        for (int u = 0; u < width; u += config_.step_cols) {
            float depth_val = depth_32f.at<float>(v, u);
            float depth_mm = depthToMillimeters(depth_val);

            if (isDepthValid(depth_mm)) {
                pcl::PointXYZ pt;
                pt.z = depth_mm;
                pt.x = static_cast<float>((u - cx) * pt.z / fx);
                pt.y = static_cast<float>((v - cy) * pt.z / fy);
                valid_points.push_back(pt);
            }
        }
    }

    // 第二步：填充点云
    cloud->width = static_cast<uint32_t>(valid_points.size());
    cloud->height = 1;  // 非组织化
    cloud->is_dense = true;
    cloud->points = valid_points;

    std::cout << "[M2] 深度图 " << width << "x" << height
              << " → 有效点云 " << cloud->size() << " 点"
              << " (过滤掉 " << (width * height - cloud->size()) << " 个无效点)"
              << std::endl;

    return true;
}

// ============================================================================
// 公开接口
// ============================================================================

bool DepthToPointCloud::depthToPointCloud(
    const cv::Mat& depth_image,
    const cv::Mat& camera_matrix,
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    if (depth_image.empty()) {
        std::cerr << "[ERROR] 深度图为空" << std::endl;
        return false;
    }

    if (camera_matrix.empty() || camera_matrix.rows != 3 || camera_matrix.cols != 3) {
        std::cerr << "[ERROR] 相机内参矩阵无效（需要3x3矩阵）" << std::endl;
        return false;
    }

    if (config_.organized) {
        return organizedDepthToCloud(depth_image, camera_matrix, cloud);
    } else {
        return unorganizedDepthToCloud(depth_image, camera_matrix, cloud);
    }
}

bool DepthToPointCloud::depthToColoredPointCloud(
    const cv::Mat& depth_image,
    const cv::Mat& color_image,
    const cv::Mat& camera_matrix,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    if (depth_image.empty() || color_image.empty()) {
        return false;
    }

    // 检查彩色图尺寸是否与深度图一致
    if (color_image.size() != depth_image.size()) {
        std::cerr << "[WARNING] 彩色图尺寸 (" << color_image.cols << "x" << color_image.rows
                  << ") 与深度图尺寸 (" << depth_image.cols << "x" << depth_image.rows
                  << ") 不一致，将缩放彩色图" << std::endl;
        cv::Mat resized_color;
        cv::resize(color_image, resized_color, depth_image.size());
        return depthToColoredPointCloud(depth_image, resized_color, camera_matrix, cloud);
    }

    // 先转换深度为XYZ点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    bool use_organized = config_.organized;
    config_.organized = true;  // 彩色点云需要组织化以保持像素对应
    bool ok = depthToPointCloud(depth_image, camera_matrix, xyz_cloud);
    config_.organized = use_organized;

    if (!ok) return false;

    // 将彩色图转换为RGB格式
    cv::Mat rgb_image;
    if (color_image.channels() == 3) {
        cv::cvtColor(color_image, rgb_image, cv::COLOR_BGR2RGB);
    } else if (color_image.channels() == 4) {
        cv::cvtColor(color_image, rgb_image, cv::COLOR_BGRA2RGB);
    } else if (color_image.channels() == 1) {
        cv::cvtColor(color_image, rgb_image, cv::COLOR_GRAY2RGB);
    } else {
        return false;
    }

    // 构造彩色点云
    cloud->width = xyz_cloud->width;
    cloud->height = xyz_cloud->height;
    cloud->is_dense = xyz_cloud->is_dense;
    cloud->points.resize(xyz_cloud->size());

    for (size_t i = 0; i < xyz_cloud->size(); ++i) {
        const auto& xyz_pt = xyz_cloud->points[i];
        auto& rgb_pt = cloud->points[i];

        rgb_pt.x = xyz_pt.x;
        rgb_pt.y = xyz_pt.y;
        rgb_pt.z = xyz_pt.z;

        if (pcl::isFinite(xyz_pt)) {
            int v = static_cast<int>(i / cloud->width);
            int u = static_cast<int>(i % cloud->width);

            if (v < rgb_image.rows && u < rgb_image.cols) {
                cv::Vec3b color = rgb_image.at<cv::Vec3b>(v, u);
                rgb_pt.r = color[0];
                rgb_pt.g = color[1];
                rgb_pt.b = color[2];
            }
        }
    }

    return true;
}

// ============================================================================
// 点云 IO
// ============================================================================

bool DepthToPointCloud::savePointCloud(
    const std::string& file_path,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    if (!cloud || cloud->empty()) {
        std::cerr << "[ERROR] 点云为空，无法保存" << std::endl;
        return false;
    }

    // 尝试移除NaN点（PCL保存时NaN点可能导致问题）
    pcl::PointCloud<pcl::PointXYZ>::Ptr clean_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud, *clean_cloud, indices);

    bool success = false;
    std::string ext = file_path.substr(file_path.find_last_of('.') + 1);
    std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);

    if (ext == "pcd") {
        // PCD格式
        int pcl_format = (config_.output_format == OutputFormat::PCD_BINARY)
                         ? pcl::io::PCD_BINARY
                         : pcl::io::PCD_ASCII;
        success = (pcl::io::savePCDFile(file_path, *clean_cloud, pcl_format) == 0);
    } else if (ext == "ply") {
        // PLY格式
        int pcl_format = (config_.output_format == OutputFormat::PLY_BINARY)
                         ? pcl::io::PLY_BINARY
                         : pcl::io::PLY_ASCII;
        success = (pcl::io::savePLYFile(file_path, *clean_cloud, pcl_format) == 0);
    } else {
        std::cerr << "[ERROR] 不支持的文件格式: " << ext << " (支持: pcd, ply)" << std::endl;
        return false;
    }

    if (success) {
        std::cout << "[M2] 点云已保存: " << file_path
                  << " (" << clean_cloud->size() << " 点)" << std::endl;
    } else {
        std::cerr << "[ERROR] 保存点云失败: " << file_path << std::endl;
    }

    return success;
}

bool DepthToPointCloud::saveColoredPointCloud(
    const std::string& file_path,
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    if (!cloud || cloud->empty()) {
        std::cerr << "[ERROR] 彩色点云为空" << std::endl;
        return false;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr clean_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud, *clean_cloud, indices);

    bool success = false;
    std::string ext = file_path.substr(file_path.find_last_of('.') + 1);
    std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);

    if (ext == "pcd") {
        int fmt = (config_.output_format == OutputFormat::PCD_BINARY)
                  ? pcl::io::PCD_BINARY : pcl::io::PCD_ASCII;
        success = (pcl::io::savePCDFile(file_path, *clean_cloud, fmt) == 0);
    } else if (ext == "ply") {
        int fmt = (config_.output_format == OutputFormat::PLY_BINARY)
                  ? pcl::io::PLY_BINARY : pcl::io::PLY_ASCII;
        success = (pcl::io::savePLYFile(file_path, *clean_cloud, fmt) == 0);
    }

    if (success) {
        std::cout << "[M2] 彩色点云已保存: " << file_path
                  << " (" << clean_cloud->size() << " 点)" << std::endl;
    }

    return success;
}

bool DepthToPointCloud::loadPointCloud(
    const std::string& file_path,
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    if (!cloud) return false;

    int ret = -1;
    std::string ext = file_path.substr(file_path.find_last_of('.') + 1);
    std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);

    if (ext == "pcd") {
        ret = pcl::io::loadPCDFile(file_path, *cloud);
    } else if (ext == "ply") {
        ret = pcl::io::loadPLYFile(file_path, *cloud);
    } else {
        return false;
    }

    if (ret == -1) {
        std::cerr << "[ERROR] 加载点云失败: " << file_path << std::endl;
        return false;
    }

    std::cout << "[M2] 点云已加载: " << file_path
              << " (" << cloud->size() << " 点)" << std::endl;
    return true;
}

// ============================================================================
// 深度图可视化
// ============================================================================

cv::Mat DepthToPointCloud::renderDepthAsColor(const cv::Mat& depth_image) {
    if (depth_image.empty()) return cv::Mat();

    cv::Mat depth_32f;
    if (depth_image.type() == CV_16UC1) {
        depth_image.convertTo(depth_32f, CV_32F, 1.0 / 1000.0);  // mm -> m
    } else if (depth_image.type() == CV_32FC1) {
        depth_32f = depth_image.clone();
    } else {
        return cv::Mat();
    }

    // 截断到有效范围（0~max_depth_mm）
    float max_m = config_.max_depth_mm / 1000.0f;
    cv::Mat normalized;
    depth_32f.convertTo(normalized, CV_8UC1, 255.0 / max_m);

    // 应用伪彩色映射
    cv::Mat color_map;
    cv::applyColorMap(normalized, color_map, cv::COLORMAP_JET);

    return color_map;
}

cv::Mat DepthToPointCloud::renderDepthColorMap(const cv::Mat& depth_image) {
    cv::Mat color_map = renderDepthAsColor(depth_image);
    if (color_map.empty()) return cv::Mat();

    // 添加颜色条和文字
    int bar_width = 30;
    int margin = 10;
    int total_width = color_map.cols + margin + bar_width + 2 * margin;

    cv::Mat display(color_map.rows + 2 * margin, total_width, CV_8UC3,
                    cv::Scalar(255, 255, 255));

    // 复制彩色图
    color_map.copyTo(display(cv::Rect(margin, margin, color_map.cols, color_map.rows)));

    // 绘制颜色条
    int bar_x = color_map.cols + 2 * margin;
    int bar_height = color_map.rows;
    for (int i = 0; i < bar_height; ++i) {
        float ratio = static_cast<float>(i) / bar_height;
        uchar val = static_cast<uchar>(255 * (1.0f - ratio));
        cv::Mat bar_color = cv::Mat(1, 1, CV_8UC3, cv::Scalar(val, val, val));
        cv::Mat bar_colored;
        cv::applyColorMap(bar_color, bar_colored, cv::COLORMAP_JET);
        cv::Scalar color = bar_colored.at<cv::Vec3b>(0, 0);
        cv::rectangle(display,
                      cv::Rect(bar_x, margin + i, bar_width, 1),
                      color, cv::FILLED);
    }

    // 添加深度标签
    std::string near_text = std::to_string(static_cast<int>(config_.min_depth_mm)) + " mm";
    std::string far_text = std::to_string(static_cast<int>(config_.max_depth_mm)) + " mm";
    cv::putText(display, near_text, cv::Point(bar_x, margin + 20),
                cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 0), 1);
    cv::putText(display, far_text, cv::Point(bar_x, margin + bar_height - 10),
                cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 0), 1);

    return display;
}

// ============================================================================
// 统计信息
// ============================================================================

DepthToPointCloud::CloudStats DepthToPointCloud::computeCloudStats(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    CloudStats stats;
    if (!cloud || cloud->empty()) return stats;

    stats.width = cloud->width;
    stats.height = cloud->height;
    stats.total_points = cloud->size();

    float min_z = std::numeric_limits<float>::max();
    float max_z = -std::numeric_limits<float>::max();
    double sum_z = 0.0;
    float min_x = std::numeric_limits<float>::max();
    float max_x = -std::numeric_limits<float>::max();
    float min_y = std::numeric_limits<float>::max();
    float max_y = -std::numeric_limits<float>::max();
    size_t valid_count = 0;

    for (const auto& pt : cloud->points) {
        if (!pcl::isFinite(pt)) {
            stats.invalid_points++;
            continue;
        }

        valid_count++;
        min_z = std::min(min_z, pt.z);
        max_z = std::max(max_z, pt.z);
        sum_z += pt.z;
        min_x = std::min(min_x, pt.x);
        max_x = std::max(max_x, pt.x);
        min_y = std::min(min_y, pt.y);
        max_y = std::max(max_y, pt.y);
    }

    stats.valid_points = valid_count;
    stats.min_depth = min_z;
    stats.max_depth = max_z;
    stats.mean_depth = static_cast<float>(sum_z / valid_count);
    stats.x_range = max_x - min_x;
    stats.y_range = max_y - min_y;
    stats.z_range = max_z - min_z;

    return stats;
}

std::string DepthToPointCloud::generateStatsReport(const CloudStats& stats) {
    std::stringstream ss;
    ss << "========================================\n";
    ss << "  点云统计报告\n";
    ss << "========================================\n\n";
    ss << "  点云尺寸: " << stats.width << " x " << stats.height << "\n";
    ss << "  总点数:   " << stats.total_points << "\n";
    ss << "  有效点:   " << stats.valid_points << "\n";
    ss << "  无效点:   " << stats.invalid_points << "\n";
    ss << "  有效比例: " << std::fixed << std::setprecision(1)
       << (100.0 * stats.valid_points / stats.total_points) << "%\n\n";
    ss << "  深度范围: " << stats.min_depth << " ~ " << stats.max_depth << " mm\n";
    ss << "  平均深度: " << stats.mean_depth << " mm\n\n";
    ss << "  X范围:    " << stats.x_range << " mm\n";
    ss << "  Y范围:    " << stats.y_range << " mm\n";
    ss << "  Z范围:    " << stats.z_range << " mm\n";
    ss << "========================================\n";
    return ss.str();
}

} // namespace bus_welding
