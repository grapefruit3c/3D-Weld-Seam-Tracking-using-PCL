#ifndef CALIBRATION_TYPES_HPP
#define CALIBRATION_TYPES_HPP

/**
 * @file calibration_types.hpp
 * @brief M1 相机标定模块 - 数据类型定义
 * @description 定义标定配置、结果、状态码等核心数据结构。
 *              所有模块共享此头文件，确保数据格式一致性。
 *
 * 集成说明（给 C# 上位机开发者）：
 * - 标定结果通过 cv::FileStorage 序列化为 YAML 文件
 * - C# 端可通过 OpenCVSharp 或直接解析 YAML 读取
 * - 推荐方案：C++ DLL 导出 loadCalibration() 给 C# 调用
 */

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <chrono>

namespace bus_welding {

// ============================================================================
// 状态码枚举
// ============================================================================
enum class CalibrationStatus {
    SUCCESS = 0,                    ///< 成功
    ERROR_IMAGE_EMPTY,              ///< 图像为空
    ERROR_CHESSBOARD_NOT_FOUND,     ///< 未检测到棋盘格
    ERROR_INSUFFICIENT_IMAGES,      ///< 有效图像不足（至少需要 10-15 张）
    ERROR_CALIBRATION_FAILED,       ///< 标定计算失败
    ERROR_FILE_IO,                  ///< 文件读写错误
    ERROR_CAMERA_OPEN_FAILED,       ///< 相机打开失败
    ERROR_INVALID_PARAMETER,        ///< 参数无效
    ERROR_NOT_CALIBRATED            ///< 尚未标定
};

/// 状态码转可读字符串
inline std::string calibrationStatusToString(CalibrationStatus status) {
    switch (status) {
        case CalibrationStatus::SUCCESS:                    return "SUCCESS";
        case CalibrationStatus::ERROR_IMAGE_EMPTY:          return "图像为空";
        case CalibrationStatus::ERROR_CHESSBOARD_NOT_FOUND: return "未检测到棋盘格";
        case CalibrationStatus::ERROR_INSUFFICIENT_IMAGES:  return "有效图像不足(至少10-15张)";
        case CalibrationStatus::ERROR_CALIBRATION_FAILED:   return "标定计算失败";
        case CalibrationStatus::ERROR_FILE_IO:              return "文件读写错误";
        case CalibrationStatus::ERROR_CAMERA_OPEN_FAILED:   return "相机打开失败";
        case CalibrationStatus::ERROR_INVALID_PARAMETER:    return "参数无效";
        case CalibrationStatus::ERROR_NOT_CALIBRATED:       return "尚未标定";
        default:                                            return "未知错误";
    }
}

// ============================================================================
// 标定配置结构体
// ============================================================================
struct CalibrationConfig {
    // ---- 棋盘格参数 ----
    int board_width  = 9;             ///< 棋盘格内角点列数 (width-1)
    int board_height = 6;             ///< 棋盘格内角点行数 (height-1)
    float square_size_mm = 30.0f;     ///< 方格边长 (mm)

    // ---- 采集参数 ----
    int min_images_required = 15;     ///< 最少需要的有效图像数
    int max_images_to_use   = 25;     ///< 最多使用的图像数
    float min_corner_confidence = 0.5;///< 角点检测置信度阈值 (0~1)

    // ---- 标定算法参数 ----
    int calibration_flags = 0;        ///< calibrateCamera flags
    /// 常用 flags 组合：
    /// cv::CALIB_USE_INTRINSIC_GUESS
    /// cv::CALIB_FIX_PRINCIPAL_POINT
    /// cv::CALIB_FIX_ASPECT_RATIO
    /// cv::CALIB_ZERO_TANGENT_DIST
    /// cv::CALIB_RATIONAL_MODEL  (用于鱼眼/广角)

    // ---- 畸变模型 ----
    // true  = 使用 Rational 模型 (8个畸变系数, k1-k6, p1-p2)
    // false = 标准模型 (5个畸变系数, k1-k3, p1-p2)
    bool use_rational_model = false;

    // ---- 亚像素角点 ----
    cv::Size subpix_window_size = cv::Size(11, 11);  ///< 亚像素搜索窗口
    cv::Size subpix_zero_zone   = cv::Size(-1, -1);  ///< 死区 (默认-1)
    cv::TermCriteria subpix_criteria = cv::TermCriteria(
        cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1);

    // ---- 输出路径 ----
    std::string output_dir = "./calibration_result";  ///< 标定结果输出目录

    /// 获取棋盘格内角点总数
    int getBoardCornerCount() const {
        return board_width * board_height;
    }

    /// 获取棋盘格内角点尺寸
    cv::Size getBoardSize() const {
        return cv::Size(board_width, board_height);
    }

    /// 获取 flags (根据 use_rational_model 自动设置)
    int getEffectiveFlags() const {
        int flags = calibration_flags;
        if (use_rational_model) {
            flags |= cv::CALIB_RATIONAL_MODEL;
        }
        return flags;
    }
};

// ============================================================================
// 单张图像检测结果
// ============================================================================
struct ImageDetectionResult {
    std::string image_path;              ///< 图像路径
    bool        success = false;         ///< 是否成功检测到棋盘格
    std::vector<cv::Point2f> corners;    ///< 检测到的角点 (像素坐标)
    cv::Size   image_size;               ///< 图像尺寸
    double     detection_time_ms = 0.0;  ///< 检测耗时 (ms)
};

// ============================================================================
// 标定结果结构体
// ============================================================================
struct CalibrationResult {
    // ---- 标定参数 ----
    cv::Mat camera_matrix;          ///< 相机内参矩阵 (3x3)
    cv::Mat distortion_coeffs;      ///< 畸变系数 (1xN)
    cv::Size image_size;            ///< 标定使用的图像尺寸

    // ---- 标定质量 ----
    double reprojection_error = 0.0;///< 重投影误差 (pixels)
    std::vector<double> per_view_errors;  ///< 每张图像的误差

    // ---- 标定参数 ----
    std::vector<cv::Mat> rvecs;     ///< 每张图像的旋转向量
    std::vector<cv::Mat> tvecs;     ///< 每张图像的平移向量

    // ---- 标定信息 ----
    int    images_used = 0;         ///< 实际使用的图像数
    int    images_total = 0;        ///< 尝试处理的图像总数
    double calibration_time_ms = 0.0;///< 标定计算耗时
    std::string timestamp;          ///< 标定时间戳

    // ---- 配置快照 ----
    CalibrationConfig config;       ///< 标定使用的配置

    // ---- 检测结果 ----
    std::vector<ImageDetectionResult> detection_results; ///< 每张图的检测结果

    // ---- 验证用 ----
    double computeMeanError() const {
        if (per_view_errors.empty()) return 0.0;
        double sum = 0.0;
        for (auto e : per_view_errors) sum += e;
        return sum / per_view_errors.size();
    }

    double computeMaxError() const {
        if (per_view_errors.empty()) return 0.0;
        return *std::max_element(per_view_errors.begin(), per_view_errors.end());
    }

    double computeStdError() const {
        if (per_view_errors.size() < 2) return 0.0;
        double mean = computeMeanError();
        double sq_sum = 0.0;
        for (auto e : per_view_errors) {
            sq_sum += (e - mean) * (e - mean);
        }
        return std::sqrt(sq_sum / (per_view_errors.size() - 1));
    }
};

// ============================================================================
// 去畸变映射（预计算，用于实时校正）
// ============================================================================
struct UndistortMaps {
    cv::Mat map1;  ///< x 映射表 (CV_32FC1 或 CV_16SC2)
    cv::Mat map2;  ///< y 映射表 (CV_32FC1 或 CV_16SC2)

    bool valid() const {
        return !map1.empty() && !map2.empty();
    }
};

// ============================================================================
// 相机信息（用于选型参考和参数传递）
// ============================================================================
struct CameraInfo {
    std::string vendor;       ///< 厂商 (Hikvision/Basler/Daheng/...)
    std::string model;        ///< 型号
    double sensor_width_mm  = 0.0;  ///< 传感器宽度 (mm)
    double sensor_height_mm = 0.0;  ///< 传感器高度 (mm)
    int pixel_width         = 0;    ///< 像素宽度
    int pixel_height        = 0;    ///< 像素高度
    double pixel_size_um    = 0.0;  ///< 像元尺寸 (um)

    /// 计算推荐标定板方格尺寸
    /// 经验公式：方格边长 ≈ 像素尺寸(um) × 10~20
    float recommendedSquareSizeMm() const {
        if (pixel_size_um > 0) {
            return pixel_size_um * 15.0f / 1000.0f;  // um -> mm
        }
        return 30.0f;  // 默认值
    }
};

} // namespace bus_welding

#endif // CALIBRATION_TYPES_HPP