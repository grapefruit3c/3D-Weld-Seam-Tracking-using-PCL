/**
 * @file camera_calibration_export.cpp
 * @brief M1 相机标定模块 - C 风格 DLL 导出实现
 * @description
 *   实现 C 风格导出函数，供 C# 上位机通过 P/Invoke 调用。
 *   这些函数包装了 CameraCalibration 类的功能，提供简洁的 C 接口。
 *
 * 注意：
 *   需要在使用此文件的 CMake 目标中定义 M1_CALIBRATION_EXPORTS 宏
 *   target_compile_definitions(M1_CalibrationLib PRIVATE M1_CALIBRATION_EXPORTS)
 */

#include "camera_calibration_export.h"
#include "camera_calibration.hpp"

#include <cstring>
#include <string>
#include <vector>

using namespace bus_welding;

// ============================================================================
// 内部辅助函数
// ============================================================================

/// 将标定结果提取到输出参数
static void extractCalibrationResult(
    const CalibrationResult& result,
    double* out_fx, double* out_fy, double* out_cx, double* out_cy,
    double* out_k1, double* out_k2, double* out_p1, double* out_p2, double* out_k3,
    double* out_reprojection_error)
{
    if (out_fx) *out_fx = result.camera_matrix.at<double>(0, 0);
    if (out_fy) *out_fy = result.camera_matrix.at<double>(1, 1);
    if (out_cx) *out_cx = result.camera_matrix.at<double>(0, 2);
    if (out_cy) *out_cy = result.camera_matrix.at<double>(1, 2);
    if (out_reprojection_error) *out_reprojection_error = result.reprojection_error;

    // 畸变系数：标准模型有 5 个 (k1,k2,p1,p2,k3)
    if (result.distortion_coeffs.total() >= 5) {
        if (out_k1) *out_k1 = result.distortion_coeffs.at<double>(0, 0);
        if (out_k2) *out_k2 = result.distortion_coeffs.at<double>(0, 1);
        if (out_p1) *out_p1 = result.distortion_coeffs.at<double>(0, 2);
        if (out_p2) *out_p2 = result.distortion_coeffs.at<double>(0, 3);
        if (out_k3) *out_k3 = result.distortion_coeffs.at<double>(0, 4);
    }
}

/// 从输出参数构建畸变系数矩阵
static cv::Mat buildDistortionCoeffs(
    double k1, double k2, double p1, double p2, double k3)
{
    return (cv::Mat_<double>(1, 5) << k1, k2, p1, p2, k3);
}

// ============================================================================
// 导出函数实现
// ============================================================================

M1_API void M1_GetVersion(char* version, int max_length) {
    std::string ver = "M1_CameraCalibration_v1.0.0";
    std::strncpy(version, ver.c_str(), max_length - 1);
    version[max_length - 1] = '\0';
}

M1_API int M1_CalibrateFromFiles(
    const char** image_paths,
    int image_count,
    const char* output_path,
    int board_width,
    int board_height,
    double square_size_mm,
    double* out_fx,
    double* out_fy,
    double* out_cx,
    double* out_cy,
    double* out_k1,
    double* out_k2,
    double* out_p1,
    double* out_p2,
    double* out_k3,
    double* out_reprojection_error)
{
    try {
        // 构建配置
        CalibrationConfig config;
        config.board_width = board_width;
        config.board_height = board_height;
        config.square_size_mm = static_cast<float>(square_size_mm);
        config.output_dir = output_path ? output_path : "./calibration_result";

        // 构建图像路径列表
        std::vector<std::string> paths;
        for (int i = 0; i < image_count; ++i) {
            if (image_paths[i]) {
                paths.push_back(image_paths[i]);
            }
        }

        // 执行标定
        CameraCalibration calibrator(config);
        CalibrationResult result = calibrator.calibrateFromFiles(paths);

        if (calibrator.getLastStatus() != CalibrationStatus::SUCCESS) {
            return -1;
        }

        // 提取结果
        extractCalibrationResult(
            result,
            out_fx, out_fy, out_cx, out_cy,
            out_k1, out_k2, out_p1, out_p2, out_k3,
            out_reprojection_error);

        return 0;

    } catch (const std::exception&) {
        return -1;
    }
}

M1_API int M1_CalibrateFromCamera(
    int camera_index,
    const char* output_path,
    int board_width,
    int board_height,
    double square_size_mm,
    double* out_fx,
    double* out_fy,
    double* out_cx,
    double* out_cy,
    double* out_k1,
    double* out_k2,
    double* out_p1,
    double* out_p2,
    double* out_k3,
    double* out_reprojection_error)
{
    try {
        // 打开相机
        OpenCVCamera camera(camera_index);
        if (!camera.open("")) {
            return -1;
        }

        // 构建配置
        CalibrationConfig config;
        config.board_width = board_width;
        config.board_height = board_height;
        config.square_size_mm = static_cast<float>(square_size_mm);
        config.output_dir = output_path ? output_path : "./calibration_result";

        // 执行标定
        CameraCalibration calibrator(config);
        CalibrationResult result = calibrator.calibrateFromCamera(&camera, true);

        camera.close();

        if (calibrator.getLastStatus() != CalibrationStatus::SUCCESS) {
            return -1;
        }

        // 提取结果
        extractCalibrationResult(
            result,
            out_fx, out_fy, out_cx, out_cy,
            out_k1, out_k2, out_p1, out_p2, out_k3,
            out_reprojection_error);

        return 0;

    } catch (const std::exception&) {
        return -1;
    }
}

M1_API int M1_LoadCalibration(
    const char* file_path,
    double* out_fx,
    double* out_fy,
    double* out_cx,
    double* out_cy,
    double* out_k1,
    double* out_k2,
    double* out_p1,
    double* out_p2,
    double* out_k3,
    double* out_reprojection_error)
{
    try {
        if (!file_path) return -1;

        CameraCalibration calibrator;
        CalibrationResult result;
        CalibrationStatus status = calibrator.loadCalibration(file_path, result);

        if (status != CalibrationStatus::SUCCESS) {
            return -1;
        }

        extractCalibrationResult(
            result,
            out_fx, out_fy, out_cx, out_cy,
            out_k1, out_k2, out_p1, out_p2, out_k3,
            out_reprojection_error);

        return 0;

    } catch (const std::exception&) {
        return -1;
    }
}

M1_API int M1_UndistortImage(
    const char* input_path,
    const char* output_path,
    double fx,
    double fy,
    double cx,
    double cy,
    double k1,
    double k2,
    double p1,
    double p2,
    double k3)
{
    try {
        if (!input_path || !output_path) return -1;

        cv::Mat image = cv::imread(input_path, cv::IMREAD_COLOR);
        if (image.empty()) return -1;

        // 构建内参矩阵和畸变系数
        cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) <<
            fx, 0, cx,
            0, fy, cy,
            0, 0, 1);

        cv::Mat dist_coeffs = buildDistortionCoeffs(k1, k2, p1, p2, k3);

        // 去畸变
        cv::Mat undistorted;
        cv::undistort(image, undistorted, camera_matrix, dist_coeffs);

        return cv::imwrite(output_path, undistorted) ? 0 : -1;

    } catch (const std::exception&) {
        return -1;
    }
}

M1_API int M1_ComputeUndistortMaps(
    double fx,
    double fy,
    double cx,
    double cy,
    double k1,
    double k2,
    double p1,
    double p2,
    double k3,
    int image_width,
    int image_height,
    const char* map_path)
{
    try {
        if (!map_path) return -1;

        cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) <<
            fx, 0, cx,
            0, fy, cy,
            0, 0, 1);

        cv::Mat dist_coeffs = buildDistortionCoeffs(k1, k2, p1, p2, k3);

        // 计算新的内参矩阵
        cv::Mat new_camera_matrix = cv::getOptimalNewCameraMatrix(
            camera_matrix, dist_coeffs,
            cv::Size(image_width, image_height), 0.0);

        // 预计算映射表
        cv::Mat map1, map2;
        cv::initUndistortRectifyMap(
            camera_matrix, dist_coeffs, cv::Mat(),
            new_camera_matrix,
            cv::Size(image_width, image_height),
            CV_16SC2, map1, map2);

        // 保存映射表
        cv::FileStorage fs(map_path, cv::FileStorage::WRITE);
        if (!fs.isOpened()) return -1;

        fs << "map1" << map1;
        fs << "map2" << map2;
        fs << "new_camera_matrix" << new_camera_matrix;
        fs.release();

        return 0;

    } catch (const std::exception&) {
        return -1;
    }
}

M1_API int M1_BatchDetectChessboard(
    const char* input_dir,
    const char* output_dir,
    int board_width,
    int board_height)
{
    try {
        if (!input_dir || !output_dir) return -1;

        CameraCalibration calibrator;
        auto config = calibrator.getConfig();
        config.board_width = board_width;
        config.board_height = board_height;
        calibrator.setConfig(config);

        std::vector<ImageDetectionResult> results;
        return calibrator.batchDetectFromDirectory(input_dir, output_dir, results);

    } catch (const std::exception&) {
        return -1;
    }
}

M1_API int M1_GenerateCalibrationReport(
    const char* calib_file,
    const char* report_path)
{
    try {
        if (!calib_file || !report_path) return -1;

        CameraCalibration calibrator;
        CalibrationResult result;
        CalibrationStatus status = calibrator.loadCalibration(calib_file, result);

        if (status != CalibrationStatus::SUCCESS) return -1;

        std::string report = calibrator.generateReport(result);

        // 保存到文件
        FILE* fp = nullptr;
        fopen_s(&fp, report_path, "w");
        if (!fp) return -1;

        fwrite(report.c_str(), 1, report.size(), fp);
        fclose(fp);

        // 同时保存质量图
        std::string quality_path = std::string(report_path);
        auto dot_pos = quality_path.find_last_of('.');
        if (dot_pos != std::string::npos) {
            quality_path = quality_path.substr(0, dot_pos) + "_quality.png";
        } else {
            quality_path += "_quality.png";
        }

        cv::Mat quality_hist = calibrator.visualizeCalibrationQuality(result);
        if (!quality_hist.empty()) {
            cv::imwrite(quality_path, quality_hist);
        }

        return 0;

    } catch (const std::exception&) {
        return -1;
    }
}