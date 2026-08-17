#ifndef CAMERA_CALIBRATION_EXPORT_H
#define CAMERA_CALIBRATION_EXPORT_H

/**
 * @file camera_calibration_export.h
 * @brief M1 相机标定模块 - C 风格 DLL 导出接口
 * @description
 *   提供纯 C 接口函数，供 C# 上位机通过 P/Invoke 调用。
 *   所有函数使用 C 链接约定 (extern "C")，支持标准 Windows DLL 导出。
 *
 * ============================================
 * C# 端 P/Invoke 调用示例
 * ============================================
 *
 * ```csharp
 * using System.Runtime.InteropServices;
 *
 * public class CameraCalibrationWrapper
 * {
 *     const string DLL_PATH = "M1_CalibrationLib.dll";
 *
 *     [DllImport(DLL_PATH, CallingConvention = CallingConvention.Cdecl)]
 *     public static extern int M1_CalibrateFromFiles(
 *         string[] imagePaths, int imageCount,
 *         string outputPath,
 *         out double fx, out double fy, out double cx, out double cy,
 *         out double reprojectionError);
 *
 *     [DllImport(DLL_PATH, CallingConvention = CallingConvention.Cdecl)]
 *     public static extern int M1_LoadCalibration(
 *         string filePath,
 *         out double fx, out double fy, out double cx, out double cy,
 *         out double k1, out double k2, out double p1, out double p2, out double k3,
 *         out double reprojectionError);
 *
 *     [DllImport(DLL_PATH, CallingConvention = CallingConvention.Cdecl)]
 *     public static extern int M1_UndistortImage(
 *         string inputPath, string outputPath,
 *         double fx, double fy, double cx, double cy,
 *         double k1, double k2, double p1, double p2, double k3);
 *
 *     [DllImport(DLL_PATH, CallingConvention = CallingConvention.Cdecl)]
 *     public static extern void M1_GetVersion(
 *         StringBuilder version, int maxLength);
 * }
 * ```
 *
 * 集成方案对比：
 *   A) C DLL + C# P/Invoke  (推荐) - 轻量级，无额外依赖
 *   B) C++/CLI 包装器        - 需 .NET 支持，桥接层代码多
 *   C) OpenCVSharp 纯 C# 实现 - 无需 C++，但性能不如 C++
 *
 * 本模块采用方案 A，提供 C 风格导出函数。
 */

#ifdef __cplusplus
extern "C" {
#endif

// 平台相关的 DLL 导出宏
#if defined(_WIN32) || defined(_WIN64)
    #ifdef M1_CALIBRATION_EXPORTS
        #define M1_API __declspec(dllexport)
    #else
        #define M1_API __declspec(dllimport)
    #endif
#else
    #define M1_API __attribute__((visibility("default")))
#endif

// ============================================================================
// 版本信息
// ============================================================================

/**
 * @brief 获取 DLL 版本号
 * @param version 输出缓冲区 (至少 64 字节)
 * @param max_length 缓冲区最大长度
 */
M1_API void M1_GetVersion(char* version, int max_length);

// ============================================================================
// 标定流程
// ============================================================================

/**
 * @brief 从图像文件列表执行标定
 * @param image_paths   图像文件路径数组
 * @param image_count   图像数量
 * @param output_path   输出目录路径
 * @param board_width   棋盘格内角点列数
 * @param board_height  棋盘格内角点行数
 * @param square_size_mm 方格边长 (mm)
 * @param out_fx        输出 fx
 * @param out_fy        输出 fy
 * @param out_cx        输出 cx
 * @param out_cy        输出 cy
 * @param out_k1        输出畸变系数 k1
 * @param out_k2        输出畸变系数 k2
 * @param out_p1        输出畸变系数 p1
 * @param out_p2        输出畸变系数 p2
 * @param out_k3        输出畸变系数 k3
 * @param out_reprojection_error 输出重投影误差
 * @return 0=成功, -1=失败
 *
 * C# 调用示例：
 * @code
 *   string[] images = Directory.GetFiles("./calib_images", "*.png");
 *   int ret = M1_CalibrateFromFiles(images, images.Length, "./result",
 *       9, 6, 30.0,
 *       out double fx, out double fy, out double cx, out double cy,
 *       out double k1, out double k2, out double p1, out double p2, out double k3,
 *       out double err);
 * @endcode
 */
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
    double* out_reprojection_error);

/**
 * @brief 从实时相机执行标定（交互模式）
 * @param camera_index  相机索引 (0, 1, 2, ...)
 * @param output_path   输出目录路径
 * @param board_width   棋盘格内角点列数
 * @param board_height  棋盘格内角点行数
 * @param square_size_mm 方格边长 (mm)
 * @param out_fx        输出 fx
 * @param out_fy        输出 fy
 * @param out_cx        输出 cx
 * @param out_cy        输出 cy
 * @param out_k1        输出畸变系数 k1
 * @param out_k2        输出畸变系数 k2
 * @param out_p1        输出畸变系数 p1
 * @param out_p2        输出畸变系数 p2
 * @param out_k3        输出畸变系数 k3
 * @param out_reprojection_error 输出重投影误差
 * @return 0=成功, -1=失败
 */
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
    double* out_reprojection_error);

// ============================================================================
// 结果加载
// ============================================================================

/**
 * @brief 从 YAML 文件加载标定结果
 * @param file_path     标定结果文件路径
 * @param out_fx        输出 fx
 * @param out_fy        输出 fy
 * @param out_cx        输出 cx
 * @param out_cy        输出 cy
 * @param out_k1        输出畸变系数 k1
 * @param out_k2        输出畸变系数 k2
 * @param out_p1        输出畸变系数 p1
 * @param out_p2        输出畸变系数 p2
 * @param out_k3        输出畸变系数 k3
 * @param out_reprojection_error 输出重投影误差
 * @return 0=成功, -1=失败
 */
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
    double* out_reprojection_error);

// ============================================================================
// 去畸变校正
// ============================================================================

/**
 * @brief 对单张图像进行去畸变校正
 * @param input_path    输入图像路径
 * @param output_path   输出图像路径
 * @param fx            相机内参 fx
 * @param fy            相机内参 fy
 * @param cx            相机内参 cx
 * @param cy            相机内参 cy
 * @param k1            畸变系数 k1
 * @param k2            畸变系数 k2
 * @param p1            畸变系数 p1
 * @param p2            畸变系数 p2
 * @param k3            畸变系数 k3
 * @return 0=成功, -1=失败
 */
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
    double k3);

/**
 * @brief 预计算去畸变映射（用于批量处理）
 * @param fx            相机内参 fx
 * @param fy            相机内参 fy
 * @param cx            相机内参 cx
 * @param cy            相机内参 cy
 * @param k1            畸变系数 k1
 * @param k2            畸变系数 k2
 * @param p1            畸变系数 p1
 * @param p2            畸变系数 p2
 * @param k3            畸变系数 k3
 * @param image_width   图像宽度
 * @param image_height  图像高度
 * @param map_path      输出映射表文件路径
 * @return 0=成功, -1=失败
 */
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
    const char* map_path);

// ============================================================================
// 批量检测
// ============================================================================

/**
 * @brief 批量检测棋盘格角点
 * @param input_dir     输入图像目录
 * @param output_dir    输出目录（标注角点的图像）
 * @param board_width   棋盘格内角点列数
 * @param board_height  棋盘格内角点行数
 * @return 成功检测的图像数量
 */
M1_API int M1_BatchDetectChessboard(
    const char* input_dir,
    const char* output_dir,
    int board_width,
    int board_height);

// ============================================================================
// 工具函数
// ============================================================================

/**
 * @brief 计算标定质量评估
 * @param calib_file    标定结果文件路径
 * @param report_path   输出报告文本文件路径
 * @return 0=成功, -1=失败
 */
M1_API int M1_GenerateCalibrationReport(
    const char* calib_file,
    const char* report_path);

#ifdef __cplusplus
}
#endif

#endif // CAMERA_CALIBRATION_EXPORT_H