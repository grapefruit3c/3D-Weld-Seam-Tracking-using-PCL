/**
 * @file main.cpp
 * @brief M1 相机标定模块 - 命令行示例程序
 * @description
 *   演示如何使用 CameraCalibration 类进行相机标定。
 *   支持三种运行模式：
 *     1. 文件模式 (-f <image_dir>)：从图像目录加载标定
 *     2. 相机模式 (-c <camera_index>)：从实时相机采集标定
 *     3. 批量检测模式 (-d <image_dir> <output_dir>)：批量检测棋盘格
 *     4. 验证模式 (-v <calib_file> <image>)：验证标定结果
 *
 * 使用示例：
 *   # 从文件标定
 *   M1_CameraCalibration.exe -f ./calib_images -o ./result
 *
 *   # 从相机标定（交互模式）
 *   M1_CameraCalibration.exe -c 0 -o ./result
 *
 *   # 验证标定结果
 *   M1_CameraCalibration.exe -v ./result/calibration_result.yml test.jpg
 */

#include "camera_calibration.hpp"
#include <iostream>
#include <filesystem>
#include <cstring>

using namespace bus_welding;

/// 打印使用帮助
void printHelp(const char* program_name) {
    std::cout << "\n========================================\n";
    std::cout << "  M1 相机内参标定模块 - 使用说明\n";
    std::cout << "========================================\n\n";

    std::cout << "用法:\n";
    std::cout << "  " << program_name << " <mode> [options]\n\n";

    std::cout << "模式:\n";
    std::cout << "  -f <image_dir> [-o <output_dir>]  从图像文件目录标定\n";
    std::cout << "  -c <camera_index> [-o <output_dir>] 从实时相机标定\n";
    std::cout << "  -d <image_dir> <output_dir>         批量检测棋盘格\n";
    std::cout << "  -v <calib_file> <image>             验证标定结果\n";
    std::cout << "  -s <calib_file>                     显示标定报告\n";
    std::cout << "  -h                                  显示此帮助\n\n";

    std::cout << "示例:\n";
    std::cout << "  " << program_name << " -f ./calib_images -o ./result\n";
    std::cout << "  " << program_name << " -c 0 -o ./result\n";
    std::cout << "  " << program_name << " -v ./result/calibration_result.yml test.jpg\n";
    std::cout << "  " << program_name << " -s ./result/calibration_result.yml\n\n";

    std::cout << "标定图像采集要求:\n";
    std::cout << "  - 棋盘格标定板: 9x6 内角点, 方格 30mm\n";
    std::cout << "  - 采集 15-25 张图像\n";
    std::cout << "  - 覆盖五个区域 (中央/左上/右上/左下/右下)\n";
    std::cout << "  - 每区域倾斜 ±15°~45°\n";
    std::cout << "  - 包含旋转变化 (0°, ±15°, ±30°)\n";
    std::cout << "========================================\n" << std::endl;
}

// ============================================================================
// 模式1：从文件标定
// ============================================================================
int runFileCalibration(const std::string& image_dir, const std::string& output_dir) {
    std::cout << "\n=== 模式: 从图像文件标定 ===\n" << std::endl;

    // 收集所有图像文件
    std::vector<std::string> image_paths;
    for (const auto& entry : std::filesystem::directory_iterator(image_dir)) {
        if (!entry.is_regular_file()) continue;

        std::string ext = entry.path().extension().string();
        std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);

        if (ext == ".jpg" || ext == ".jpeg" || ext == ".png" ||
            ext == ".bmp" || ext == ".tiff" || ext == ".tif") {
            image_paths.push_back(entry.path().string());
        }
    }

    if (image_paths.empty()) {
        std::cerr << "[ERROR] 目录中未找到图像文件: " << image_dir << std::endl;
        return -1;
    }

    std::sort(image_paths.begin(), image_paths.end());
    std::cout << "找到 " << image_paths.size() << " 张图像\n" << std::endl;

    // 配置标定参数
    CalibrationConfig config;
    config.board_width = 9;          // 内角点列数
    config.board_height = 6;         // 内角点行数
    config.square_size_mm = 30.0f;   // 方格边长 (mm)
    config.min_images_required = 10; // 最少有效图像数
    config.max_images_to_use = 25;   // 最多使用图像数
    config.output_dir = output_dir;

    // 创建标定器并执行标定
    CameraCalibration calibrator(config);

    auto start_time = std::chrono::high_resolution_clock::now();
    CalibrationResult result = calibrator.calibrateFromFiles(image_paths);
    auto end_time = std::chrono::high_resolution_clock::now();

    double total_time = std::chrono::duration<double, std::milli>(
        end_time - start_time).count();

    // 输出结果
    if (calibrator.getLastStatus() == CalibrationStatus::SUCCESS) {
        std::cout << "\n" << calibrator.generateReport(result) << std::endl;
        std::cout << "总耗时: " << std::fixed << std::setprecision(1)
                  << total_time << " ms" << std::endl;
        std::cout << "标定结果已保存至: " << output_dir << std::endl;

        // 可视化标定质量
        cv::Mat quality_hist = calibrator.visualizeCalibrationQuality(result);
        if (!quality_hist.empty()) {
            cv::imwrite(output_dir + "/calibration_quality.png", quality_hist);
            std::cout << "质量图已保存: " << output_dir
                      << "/calibration_quality.png" << std::endl;
        }

        return 0;
    } else {
        std::cerr << "[ERROR] 标定失败: "
                  << calibrationStatusToString(calibrator.getLastStatus())
                  << " - " << calibrator.getLastErrorMessage() << std::endl;
        return -1;
    }
}

// ============================================================================
// 模式2：从相机标定
// ============================================================================
int runCameraCalibration(int camera_index, const std::string& output_dir) {
    std::cout << "\n=== 模式: 从实时相机标定 ===\n" << std::endl;

    // 打开相机
    OpenCVCamera camera(camera_index);
    if (!camera.open("")) {
        std::cerr << "[ERROR] 无法打开相机: " << camera_index << std::endl;
        return -1;
    }

    std::cout << "相机已打开: " << camera.getCameraName() << std::endl;
    std::cout << "图像尺寸: " << camera.getImageSize().width << " x "
              << camera.getImageSize().height << std::endl;

    // 配置标定参数
    CalibrationConfig config;
    config.board_width = 9;
    config.board_height = 6;
    config.square_size_mm = 30.0f;
    config.min_images_required = 15;
    config.max_images_to_use = 25;
    config.output_dir = output_dir;

    // 创建标定器
    CameraCalibration calibrator(config);

    // 执行标定（交互模式，按空格键保存图像）
    CalibrationResult result = calibrator.calibrateFromCamera(&camera, true);

    camera.close();

    // 输出结果
    if (calibrator.getLastStatus() == CalibrationStatus::SUCCESS) {
        std::cout << "\n" << calibrator.generateReport(result) << std::endl;
        return 0;
    } else {
        std::cerr << "[ERROR] 标定失败: "
                  << calibrationStatusToString(calibrator.getLastStatus())
                  << std::endl;
        return -1;
    }
}

// ============================================================================
// 模式3：批量检测棋盘格
// ============================================================================
int runBatchDetect(const std::string& input_dir, const std::string& output_dir) {
    std::cout << "\n=== 模式: 批量检测棋盘格 ===\n" << std::endl;

    CameraCalibration calibrator;

    std::vector<ImageDetectionResult> results;
    int success = calibrator.batchDetectFromDirectory(input_dir, output_dir, results);

    std::cout << "检测结果: " << success << " / " << results.size() << " 成功" << std::endl;

    for (const auto& r : results) {
        std::string status = r.success ? "[OK]" : "[FAIL]";
        std::cout << status << " " << r.image_path;
        if (r.success) {
            std::cout << " (" << r.detection_time_ms << " ms)";
        }
        std::cout << std::endl;
    }

    return 0;
}

// ============================================================================
// 模式4：验证标定结果
// ============================================================================
int runVerification(const std::string& calib_file, const std::string& test_image) {
    std::cout << "\n=== 模式: 验证标定结果 ===\n" << std::endl;

    // 加载标定结果
    CameraCalibration calibrator;
    CalibrationResult result;
    CalibrationStatus status = calibrator.loadCalibration(calib_file, result);

    if (status != CalibrationStatus::SUCCESS) {
        std::cerr << "[ERROR] 加载标定文件失败: " << calib_file << std::endl;
        return -1;
    }

    std::cout << "标定结果加载成功" << std::endl;
    std::cout << "重投影误差: " << result.reprojection_error << " px" << std::endl;
    std::cout << "图像尺寸: " << result.image_size.width << " x "
              << result.image_size.height << std::endl;

    // 加载测试图像
    cv::Mat test_img = cv::imread(test_image, cv::IMREAD_COLOR);
    if (test_img.empty()) {
        std::cerr << "[ERROR] 无法读取测试图像: " << test_image << std::endl;
        return -1;
    }

    // 方式1：使用预计算映射表（推荐，性能最优）
    std::cout << "\n方式1: 预计算映射表 + remap 去畸变..." << std::endl;
    UndistortMaps maps = calibrator.computeUndistortMaps(
        result.camera_matrix, result.distortion_coeffs, result.image_size);

    cv::Mat undistorted1;
    auto t1_start = std::chrono::high_resolution_clock::now();
    calibrator.undistortImage(test_img, undistorted1, maps);
    auto t1_end = std::chrono::high_resolution_clock::now();
    double t1 = std::chrono::duration<double, std::milli>(t1_end - t1_start).count();

    // 方式2：直接 undistort（方便，适合单次使用）
    std::cout << "方式2: 直接 undistort 去畸变..." << std::endl;
    cv::Mat undistorted2;
    auto t2_start = std::chrono::high_resolution_clock::now();
    calibrator.undistortImageDirect(test_img, undistorted2,
                                     result.camera_matrix, result.distortion_coeffs);
    auto t2_end = std::chrono::high_resolution_clock::now();
    double t2 = std::chrono::duration<double, std::milli>(t2_end - t2_start).count();

    std::cout << "\n性能对比:" << std::endl;
    std::cout << "  方式1 (remap): " << std::fixed << std::setprecision(2) << t1 << " ms" << std::endl;
    std::cout << "  方式2 (undistort): " << t2 << " ms" << std::endl;
    std::cout << "  加速比: " << std::fixed << std::setprecision(1) << (t2 / t1) << "x" << std::endl;

    // 保存对比图
    cv::Mat comparison = calibrator.visualizeUndistortion(test_img, maps);
    if (!comparison.empty()) {
        std::string output_path = std::filesystem::path(test_image).parent_path().string()
            + "/undistortion_comparison.png";
        cv::imwrite(output_path, comparison);
        std::cout << "\n去畸变对比图已保存: " << output_path << std::endl;
    }

    // 显示结果
    cv::imshow("Original (Distorted)", test_img);
    cv::imshow("Undistorted", undistorted1);
    if (!comparison.empty()) {
        cv::imshow("Comparison", comparison);
    }
    std::cout << "\n按任意键退出..." << std::endl;
    cv::waitKey(0);
    cv::destroyAllWindows();

    return 0;
}

// ============================================================================
// 模式5：显示标定报告
// ============================================================================
int runShowReport(const std::string& calib_file) {
    std::cout << "\n=== 模式: 显示标定报告 ===\n" << std::endl;

    CameraCalibration calibrator;
    CalibrationResult result;
    CalibrationStatus status = calibrator.loadCalibration(calib_file, result);

    if (status != CalibrationStatus::SUCCESS) {
        std::cerr << "[ERROR] 加载标定文件失败: " << calib_file << std::endl;
        return -1;
    }

    std::cout << calibrator.generateReport(result) << std::endl;

    // 生成并保存质量图
    cv::Mat quality_hist = calibrator.visualizeCalibrationQuality(result);
    if (!quality_hist.empty()) {
        std::string output_path = std::filesystem::path(calib_file).parent_path().string()
            + "/calibration_quality.png";
        cv::imwrite(output_path, quality_hist);
        std::cout << "质量图已保存: " << output_path << std::endl;
        cv::imshow("Calibration Quality", quality_hist);
        cv::waitKey(0);
    }

    return 0;
}

// ============================================================================
// 主函数
// ============================================================================
int main(int argc, char** argv) {
    std::cout << "========================================\n";
    std::cout << "  M1 相机内参标定模块 v1.0.0\n";
    std::cout << "  客车焊接 3D 视觉引导系统\n";
    std::cout << "========================================\n" << std::endl;

    if (argc < 2) {
        printHelp(argv[0]);
        return 0;
    }

    std::string mode = argv[1];

    try {
        if (mode == "-h" || mode == "--help") {
            printHelp(argv[0]);
            return 0;
        }
        else if (mode == "-f" && argc >= 3) {
            // 从文件标定
            std::string image_dir = argv[2];
            std::string output_dir = (argc >= 4 && std::string(argv[3]) == "-o")
                                     ? argv[4] : "./calibration_result";
            return runFileCalibration(image_dir, output_dir);
        }
        else if (mode == "-c" && argc >= 3) {
            // 从相机标定
            int camera_index = std::stoi(argv[2]);
            std::string output_dir = (argc >= 4 && std::string(argv[3]) == "-o")
                                     ? argv[4] : "./calibration_result";
            return runCameraCalibration(camera_index, output_dir);
        }
        else if (mode == "-d" && argc >= 4) {
            // 批量检测
            return runBatchDetect(argv[2], argv[3]);
        }
        else if (mode == "-v" && argc >= 4) {
            // 验证标定结果
            return runVerification(argv[2], argv[3]);
        }
        else if (mode == "-s" && argc >= 3) {
            // 显示标定报告
            return runShowReport(argv[2]);
        }
        else {
            std::cerr << "[ERROR] 无效的参数或参数不足" << std::endl;
            printHelp(argv[0]);
            return -1;
        }
    }
    catch (const std::exception& e) {
        std::cerr << "[ERROR] 程序异常: " << e.what() << std::endl;
        return -1;
    }
}