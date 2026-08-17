#ifndef CAMERA_CALIBRATION_HPP
#define CAMERA_CALIBRATION_HPP

/**
 * @file camera_calibration.hpp
 * @brief M1 相机内参标定模块 - 核心类
 * @description
 *   相机内参标定是 3D 视觉系统的基石。
 *   本模块实现基于棋盘格标定板的相机内参标定和畸变校正。
 *
 * 标定流程：
 *   1. 采集 15-25 张不同位姿的棋盘格图像
 *   2. 检测棋盘格角点 (亚像素精度)
 *   3. 执行相机标定 (calibrateCamera)
 *   4. 评估标定质量 (重投影误差)
 *   5. 保存标定结果
 *   6. 预计算去畸变映射 (用于实时校正)
 *
 * 验收标准：
 *   - 重投影误差 < 0.15 pixel (推荐)
 *   - 重投影误差 < 0.30 pixel (可接受)
 *   - 重投影误差 > 0.50 pixel (需要重新标定)
 *
 * 集成到 WPF 的方式：
 *   本类编译为 C++ DLL，通过 C 风格导出函数供 C# 调用。
 *   C# 端通过 P/Invoke 或 C++/CLI 包装器调用。
 *
 * 相机 SDK 选型建议：
 *   - 推荐海康威视 (Hikvision) 工业相机
 *     - SDK 完善，C++ 示例丰富
 *     - 支持 GigE/USB3 接口
 *     - 与机器人视觉引导系统兼容性好
 *   - 备选方案：Basler (Pylon SDK)、大恒 (Galaxy SDK)
 *   - 低成本方案：USB 相机 + OpenCV VideoCapture
 */

#include "calibration_types.hpp"
#include "camera_interface.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>

#include <string>
#include <vector>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <ctime>

namespace bus_welding {

/**
 * @brief 相机内参标定器
 *
 * 核心功能：
 * 1. 从图像文件或实时相机进行标定
 * 2. 检测棋盘格角点（带亚像素优化）
 * 3. 执行 OpenCV 标定算法
 * 4. 保存/加载标定结果
 * 5. 预计算去畸变映射（实时校正用）
 * 6. 标定质量评估与可视化
 *
 * 线程安全：
 * - 标定过程本身是单线程操作
 * - 去畸变映射可以多线程并行处理
 */
class CameraCalibration {
public:
    /// 默认构造函数，使用默认配置
    CameraCalibration();

    /// 使用自定义配置构造
    explicit CameraCalibration(const CalibrationConfig& config);

    ~CameraCalibration() = default;

    // 禁止拷贝
    CameraCalibration(const CameraCalibration&) = delete;
    CameraCalibration& operator=(const CameraCalibration&) = delete;

    // 允许移动
    CameraCalibration(CameraCalibration&&) = default;
    CameraCalibration& operator=(CameraCalibration&&) = default;

    // ========================================================================
    // 配置管理
    // ========================================================================

    /// 获取当前配置
    const CalibrationConfig& getConfig() const { return config_; }

    /// 更新配置
    void setConfig(const CalibrationConfig& config) { config_ = config; }

    // ========================================================================
    // 核心标定流程
    // ========================================================================

    /**
     * @brief 从图像文件列表进行标定
     * @param image_paths 棋盘格图像文件路径列表
     * @return 标定结果（含状态码和详细数据）
     *
     * 使用示例：
     * @code
     *   CameraCalibration calib;
     *   std::vector<std::string> images = {"img1.jpg", "img2.jpg", ...};
     *   CalibrationResult result = calib.calibrateFromFiles(images);
     *   if (result.status == CalibrationStatus::SUCCESS) {
     *       std::cout << "重投影误差: " << result.reprojection_error << " px\n";
     *   }
     * @endcode
     */
    CalibrationResult calibrateFromFiles(const std::vector<std::string>& image_paths);

    /**
     * @brief 从实时相机进行标定
     * @param camera 相机接口指针
     * @param interactive 是否交互模式（显示实时预览和按键保存）
     * @return 标定结果
     *
     * 交互模式按键：
     *   SPACE - 保存当前帧用于标定
     *   ENTER - 完成采集，开始标定
     *   ESC   - 退出
     *   R     - 重置已采集的图像
     */
    CalibrationResult calibrateFromCamera(CameraInterface* camera, bool interactive = true);

    /**
     * @brief 单张图像棋盘格检测
     * @param image 输入图像 (BGR 或 灰度)
     * @param corners 输出角点 (像素坐标)
     * @return true 检测成功
     */
    bool detectChessboard(const cv::Mat& image, std::vector<cv::Point2f>& corners);

    // ========================================================================
    // 结果持久化
    // ========================================================================

    /**
     * @brief 保存标定结果到 YAML 文件
     * @param result 标定结果
     * @param filepath 输出文件路径（推荐 .yml 或 .yaml 扩展名）
     * @return 状态码
     *
     * 文件格式说明（C# 端可通过 OpenCVSharp 或 YAML 解析库读取）：
     * @code
     * %YAML:1.0
     * ---
     * camera_matrix: !!opencv-matrix
     *   rows: 3
     *   cols: 3
     *   dt: d
     *   data: [ fx, 0, cx, 0, fy, cy, 0, 0, 1 ]
     * distortion_coeffs: !!opencv-matrix
     *   rows: 1
     *   cols: 5
     *   dt: d
     *   data: [ k1, k2, p1, p2, k3 ]
     * reprojection_error: 0.123
     * image_width: 2448
     * image_height: 2048
     * @endcode
     */
    CalibrationStatus saveCalibration(const CalibrationResult& result, const std::string& filepath);

    /**
     * @brief 从 YAML 文件加载标定结果
     * @param filepath 标定结果文件路径
     * @param result 输出标定结果
     * @return 状态码
     */
    CalibrationStatus loadCalibration(const std::string& filepath, CalibrationResult& result);

    // ========================================================================
    // 去畸变校正
    // ========================================================================

    /**
     * @brief 预计算去畸变映射（用于实时校正）
     * @param camera_matrix 相机内参
     * @param dist_coeffs 畸变系数
     * @param image_size 图像尺寸
     * @param alpha 缩放参数 (0=保留全部有效像素, 1=保留全部原始像素)
     * @return 去畸变映射
     *
     * alpha 参数说明：
     *   - alpha=0: 裁剪掉黑色区域，图像尺寸可能变小
     *   - alpha=1: 保留所有原始像素，图像尺寸不变但有黑色区域
     *   推荐 alpha=0 以最大化有效像素区域
     */
    UndistortMaps computeUndistortMaps(
        const cv::Mat& camera_matrix,
        const cv::Mat& dist_coeffs,
        const cv::Size& image_size,
        double alpha = 0.0);

    /**
     * @brief 对单张图像进行去畸变校正
     * @param distorted 畸变图像
     * @param undistorted 输出校正后图像
     * @param maps 预计算的映射表
     * @return true 成功
     */
    bool undistortImage(
        const cv::Mat& distorted,
        cv::Mat& undistorted,
        const UndistortMaps& maps);

    /**
     * @brief 便捷方法：直接对图像进行去畸变
     * @param distorted 畸变图像
     * @param undistorted 输出校正后图像
     * @param camera_matrix 相机内参
     * @param dist_coeffs 畸变系数
     * @return true 成功
     */
    bool undistortImageDirect(
        const cv::Mat& distorted,
        cv::Mat& undistorted,
        const cv::Mat& camera_matrix,
        const cv::Mat& dist_coeffs);

    // ========================================================================
    // 质量评估与可视化
    // ========================================================================

    /**
     * @brief 计算标定质量报告
     * @param result 标定结果
     * @return 格式化的报告字符串
     *
     * 报告内容包含：
     * - 总体重投影误差
     * - 每张图像的误差分布
     * - 内参矩阵
     * - 畸变系数
     * - 标定建议
     */
    std::string generateReport(const CalibrationResult& result);

    /**
     * @brief 绘制检测到的棋盘格角点
     * @param image 输入图像（会绘制在图像上）
     * @param corners 检测到的角点
     * @param pattern_size 棋盘格内角点尺寸
     * @param found 是否成功检测
     */
    void drawDetectedCorners(
        cv::Mat& image,
        const std::vector<cv::Point2f>& corners,
        const cv::Size& pattern_size,
        bool found);

    /**
     * @brief 可视化标定结果（误差分布直方图）
     * @param result 标定结果
     * @return 可视化图像
     */
    cv::Mat visualizeCalibrationQuality(const CalibrationResult& result);

    /**
     * @brief 可视化去畸变前后对比
     * @param original 原始图像
     * @param maps 去畸变映射
     * @return 对比图（左右拼接）
     */
    cv::Mat visualizeUndistortion(
        const cv::Mat& original,
        const UndistortMaps& maps);

    // ========================================================================
    // 批量处理
    // ========================================================================

    /**
     * @brief 批量检测目录中的所有图像
     * @param input_dir 输入目录（支持 .jpg, .png, .bmp, .tiff）
     * @param output_dir 输出目录（绘制了角点的图像）
     * @param corner_output 输出检测到的有效角点列表
     * @return 成功检测的图像数量
     */
    int batchDetectFromDirectory(
        const std::string& input_dir,
        const std::string& output_dir,
        std::vector<ImageDetectionResult>& corner_output);

    // ========================================================================
    // 工具方法
    // ========================================================================

    /// 获取当前标定器状态
    CalibrationStatus getLastStatus() const { return last_status_; }

    /// 获取最后错误信息
    std::string getLastErrorMessage() const { return last_error_message_; }

    /// 获取当前库版本
    static std::string getVersion() { return "M1_CameraCalibration_v1.0.0"; }

    /// 生成时间戳字符串
    static std::string getCurrentTimestamp();

private:
    /// 配置
    CalibrationConfig config_;

    /// 最后状态
    CalibrationStatus last_status_ = CalibrationStatus::SUCCESS;

    /// 最后错误信息
    std::string last_error_message_;

    /// 存储当前采集的图像（用于实时标定）
    std::vector<cv::Mat> captured_images_;

    /// 内部：棋盘格在世界坐标系中的 3D 点
    std::vector<cv::Point3f> generateBoardPoints() const;

    /// 内部：设置错误状态
    void setError(CalibrationStatus status, const std::string& message);

    /// 内部：确保输出目录存在
    bool ensureDirectory(const std::string& dir_path);

    /// 内部：重投影误差计算
    double computeReprojectionError(
        const std::vector<std::vector<cv::Point2f>>& image_points,
        const std::vector<std::vector<cv::Point3f>>& object_points,
        const cv::Mat& camera_matrix,
        const cv::Mat& dist_coeffs,
        const std::vector<cv::Mat>& rvecs,
        const std::vector<cv::Mat>& tvecs,
        std::vector<double>& per_view_errors);
};

} // namespace bus_welding

#endif // CAMERA_CALIBRATION_HPP