/**
 * @file camera_calibration.cpp
 * @brief M1 相机内参标定模块 - 实现
 * @description
 *   基于 OpenCV 实现棋盘格相机标定，包括：
 *   - 棋盘格角点检测（亚像素精度）
 *   - Zhang 标定法 (calibrateCamera)
 *   - 重投影误差评估
 *   - 去畸变映射预计算
 *   - 标定结果持久化
 *
 * 算法原理：
 *   相机标定本质是求解投影矩阵 P = K[R|t] 的过程。
 *   通过已知 3D 点（棋盘格）和图像 2D 点之间的对应关系，
 *   使用最小二乘法优化求解内参矩阵 K 和畸变系数。
 *
 * 参考：
 *   - Zhang Z. "A Flexible New Technique for Camera Calibration" (2000)
 *   - OpenCV: calibrateCamera(), findChessboardCorners()
 */

#include "camera_calibration.hpp"

#include <algorithm>
#include <numeric>
#include <sstream>

namespace bus_welding {

// ============================================================================
// 构造函数
// ============================================================================

CameraCalibration::CameraCalibration()
    : config_(CalibrationConfig()) {
}

CameraCalibration::CameraCalibration(const CalibrationConfig& config)
    : config_(config) {
}

// ============================================================================
// 内部工具方法
// ============================================================================

void CameraCalibration::setError(CalibrationStatus status, const std::string& message) {
    last_status_ = status;
    last_error_message_ = message;
}

std::string CameraCalibration::getCurrentTimestamp() {
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d %H:%M:%S");
    return ss.str();
}

bool CameraCalibration::ensureDirectory(const std::string& dir_path) {
    try {
        std::filesystem::create_directories(dir_path);
        return true;
    } catch (const std::exception& e) {
        setError(CalibrationStatus::ERROR_FILE_IO, 
                 "创建目录失败: " + std::string(e.what()));
        return false;
    }
}

/**
 * 生成棋盘格标定板的 3D 世界坐标点。
 * 假设棋盘格位于 Z=0 平面，角点按行优先排列。
 * 坐标原点在棋盘格左上角第一个内角点。
 */
std::vector<cv::Point3f> CameraCalibration::generateBoardPoints() const {
    std::vector<cv::Point3f> points;
    points.reserve(config_.getBoardCornerCount());

    for (int row = 0; row < config_.board_height; ++row) {
        for (int col = 0; col < config_.board_width; ++col) {
            // 世界坐标：X 向右，Y 向下，Z=0
            points.emplace_back(
                col * config_.square_size_mm,   // X
                row * config_.square_size_mm,   // Y
                0.0f);                          // Z
        }
    }
    return points;
}

// ============================================================================
// 棋盘格角点检测
// ============================================================================

bool CameraCalibration::detectChessboard(
    const cv::Mat& image,
    std::vector<cv::Point2f>& corners)
{
    if (image.empty()) {
        setError(CalibrationStatus::ERROR_IMAGE_EMPTY, "输入图像为空");
        return false;
    }

    cv::Mat gray;
    if (image.channels() == 3) {
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    } else if (image.channels() == 4) {
        cv::cvtColor(image, gray, cv::COLOR_BGRA2GRAY);
    } else {
        gray = image.clone();
    }

    // 第一步：使用亚像素精度检测棋盘格角点
    // CALIB_CB_ADAPTIVE_THRESH: 自适应阈值二值化，适应光照变化
    // CALIB_CB_NORMALIZE_IMAGE: 归一化图像亮度，提高检测鲁棒性
    // CALIB_CB_FAST_CHECK: 快速预检查，跳过明显不是棋盘格的图像
    int chessboard_flags = cv::CALIB_CB_ADAPTIVE_THRESH
                         | cv::CALIB_CB_NORMALIZE_IMAGE
                         | cv::CALIB_CB_FAST_CHECK;

    bool found = cv::findChessboardCorners(
        gray,
        config_.getBoardSize(),
        corners,
        chessboard_flags);

    if (!found) {
        return false;
    }

    // 第二步：亚像素精细化（提高角点精度至亚像素级别）
    // 这是达到 ±0.5mm 焊接定位精度的关键步骤
    cv::cornerSubPix(
        gray,
        corners,
        config_.subpix_window_size,   // 搜索窗口大小
        config_.subpix_zero_zone,     // 死区（-1 表示无死区）
        config_.subpix_criteria);     // 迭代终止条件

    return true;
}

void CameraCalibration::drawDetectedCorners(
    cv::Mat& image,
    const std::vector<cv::Point2f>& corners,
    const cv::Size& pattern_size,
    bool found)
{
    cv::drawChessboardCorners(image, pattern_size, corners, found);
}

// ============================================================================
// 重投影误差计算
// ============================================================================

double CameraCalibration::computeReprojectionError(
    const std::vector<std::vector<cv::Point2f>>& image_points,
    const std::vector<std::vector<cv::Point3f>>& object_points,
    const cv::Mat& camera_matrix,
    const cv::Mat& dist_coeffs,
    const std::vector<cv::Mat>& rvecs,
    const std::vector<cv::Mat>& tvecs,
    std::vector<double>& per_view_errors)
{
    per_view_errors.clear();
    per_view_errors.reserve(image_points.size());

    double total_error = 0.0;
    int total_points = 0;

    for (size_t i = 0; i < image_points.size(); ++i) {
        // 将 3D 点投影到图像平面
        std::vector<cv::Point2f> projected_points;
        cv::projectPoints(
            object_points[i],
            rvecs[i],
            tvecs[i],
            camera_matrix,
            dist_coeffs,
            projected_points);

        // 计算投影点与检测点之间的 RMS 误差
        double error = cv::norm(image_points[i], projected_points, cv::NORM_L2SQR);
        error /= static_cast<double>(image_points[i].size());
        error = std::sqrt(error);  // RMS

        per_view_errors.push_back(error);
        total_error += error * image_points[i].size();
        total_points += static_cast<int>(image_points[i].size());
    }

    if (total_points == 0) return 0.0;
    return total_error / total_points;
}

// ============================================================================
// 核心标定流程
// ============================================================================

CalibrationResult CameraCalibration::calibrateFromFiles(
    const std::vector<std::string>& image_paths)
{
    auto start_time = std::chrono::high_resolution_clock::now();

    CalibrationResult result;
    result.config = config_;
    result.timestamp = getCurrentTimestamp();
    result.images_total = static_cast<int>(image_paths.size());

    // ---- 第一步：加载图像并检测棋盘格角点 ----
    std::vector<std::vector<cv::Point2f>> all_image_points;
    std::vector<std::vector<cv::Point3f>> all_object_points;
    std::vector<cv::Mat> valid_images;
    cv::Size image_size;

    // 生成棋盘格 3D 点（每张图像相同，因为棋盘格是刚体）
    std::vector<cv::Point3f> board_points = generateBoardPoints();

    for (size_t i = 0; i < image_paths.size(); ++i) {
        const auto& img_path = image_paths[i];

        // 记录检测结果
        ImageDetectionResult det_result;
        det_result.image_path = img_path;

        auto detect_start = std::chrono::high_resolution_clock::now();

        cv::Mat image = cv::imread(img_path, cv::IMREAD_COLOR);
        if (image.empty()) {
            det_result.success = false;
            result.detection_results.push_back(det_result);
            std::cerr << "[WARNING] 无法读取图像: " << img_path << std::endl;
            continue;
        }

        // 记录图像尺寸（所有图像尺寸应一致）
        if (image_size == cv::Size(0, 0)) {
            image_size = image.size();
        } else if (image.size() != image_size) {
            std::cerr << "[WARNING] 图像尺寸不一致: " << img_path
                      << " (期望: " << image_size << ", 实际: " << image.size() << ")"
                      << std::endl;
            // 仍然继续，但记录警告
        }

        // 检测棋盘格角点
        std::vector<cv::Point2f> corners;
        bool found = detectChessboard(image, corners);

        auto detect_end = std::chrono::high_resolution_clock::now();
        det_result.detection_time_ms = std::chrono::duration<double, std::milli>(
            detect_end - detect_start).count();

        if (found) {
            det_result.success = true;
            det_result.corners = corners;
            det_result.image_size = image.size();

            all_image_points.push_back(corners);
            all_object_points.push_back(board_points);
            valid_images.push_back(image);

            std::cout << "[OK] 检测到棋盘格: " << img_path
                      << " (角点数: " << corners.size() << ")"
                      << std::endl;
        } else {
            det_result.success = false;
            std::cout << "[FAIL] 未检测到棋盘格: " << img_path << std::endl;
        }

        result.detection_results.push_back(det_result);
    }

    // ---- 第二步：检查有效图像数量 ----
    int valid_count = static_cast<int>(all_image_points.size());
    if (valid_count < config_.min_images_required) {
        setError(CalibrationStatus::ERROR_INSUFFICIENT_IMAGES,
                 "有效图像不足: " + std::to_string(valid_count) +
                 " (需要至少 " + std::to_string(config_.min_images_required) + " 张)");
        result.reprojection_error = -1.0;
        return result;
    }

    // 如果图像过多，只使用部分图像（分散选择保证覆盖不同位姿）
    if (valid_count > config_.max_images_to_use) {
        std::cout << "[INFO] 有效图像 " << valid_count << " 张，将选择 "
                  << config_.max_images_to_use << " 张进行标定" << std::endl;

        std::vector<std::vector<cv::Point2f>> selected_points;
        std::vector<std::vector<cv::Point3f>> selected_object_points;

        int step = valid_count / config_.max_images_to_use;
        if (step < 1) step = 1;

        for (int i = 0; i < valid_count && selected_points.size() < static_cast<size_t>(config_.max_images_to_use); i += step) {
            selected_points.push_back(all_image_points[i]);
            selected_object_points.push_back(all_object_points[i]);
        }

        all_image_points = selected_points;
        all_object_points = selected_object_points;
    }

    // ---- 第三步：执行相机标定 ----
    std::cout << "\n开始标定计算，使用 " << all_image_points.size() << " 张图像..."
              << std::endl;

    cv::Mat camera_matrix = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat dist_coeffs;
    std::vector<cv::Mat> rvecs, tvecs;

    double reproj_error = cv::calibrateCamera(
        all_object_points,          // 世界坐标系 3D 点
        all_image_points,           // 图像坐标系 2D 点
        image_size,                 // 图像尺寸
        camera_matrix,              // 输出：内参矩阵
        dist_coeffs,                // 输出：畸变系数
        rvecs,                      // 输出：每张图像的旋转向量
        tvecs,                      // 输出：每张图像的平移向量
        config_.getEffectiveFlags(), // 标定 flags
        cv::TermCriteria(
            cv::TermCriteria::EPS + cv::TermCriteria::COUNT,
            100,    // 最大迭代次数
            1e-6)); // 精度要求

    // ---- 第四步：计算重投影误差 ----
    std::vector<double> per_view_errors;
    double final_error = computeReprojectionError(
        all_image_points, all_object_points,
        camera_matrix, dist_coeffs,
        rvecs, tvecs, per_view_errors);

    // ---- 第五步：填充结果 ----
    result.camera_matrix = camera_matrix.clone();
    result.distortion_coeffs = dist_coeffs.clone();
    result.image_size = image_size;
    result.reprojection_error = final_error;
    result.per_view_errors = per_view_errors;
    result.rvecs = rvecs;
    result.tvecs = tvecs;
    result.images_used = static_cast<int>(all_image_points.size());

    auto end_time = std::chrono::high_resolution_clock::now();
    result.calibration_time_ms = std::chrono::duration<double, std::milli>(
        end_time - start_time).count();

    // ---- 第六步：保存结果 ----
    std::string output_file = config_.output_dir + "/calibration_result.yml";
    if (ensureDirectory(config_.output_dir)) {
        saveCalibration(result, output_file);
    }

    setError(CalibrationStatus::SUCCESS, "标定成功");
    return result;
}

CalibrationResult CameraCalibration::calibrateFromCamera(
    CameraInterface* camera,
    bool interactive)
{
    if (!camera || !camera->isOpen()) {
        CalibrationResult result;
        result.config = config_;
        result.timestamp = getCurrentTimestamp();
        setError(CalibrationStatus::ERROR_CAMERA_OPEN_FAILED, "相机未打开");
        return result;
    }

    captured_images_.clear();

    if (!interactive) {
        // 自动采集：连续采集直到达到最少图像数
        std::cout << "自动采集模式：采集 " << config_.min_images_required
                  << " 张有效图像..." << std::endl;

        int valid_count = 0;
        while (valid_count < config_.min_images_required) {
            cv::Mat frame;
            if (!camera->grabFrame(frame)) {
                std::cerr << "[ERROR] 采集图像失败" << std::endl;
                continue;
            }

            std::vector<cv::Point2f> corners;
            if (detectChessboard(frame, corners)) {
                captured_images_.push_back(frame.clone());
                valid_count++;
                std::cout << "\r已采集: " << valid_count << "/"
                          << config_.min_images_required << std::endl;
            }

            // 短暂延时，避免 CPU 空转
            cv::waitKey(200);
        }
    } else {
        // 交互模式：用户按空格键保存图像，回车完成
        std::cout << "\n=== 交互式标定采集 ===" << std::endl;
        std::cout << "操作说明:" << std::endl;
        std::cout << "  SPACE - 保存当前帧用于标定" << std::endl;
        std::cout << "  ENTER - 完成采集，开始标定" << std::endl;
        std::cout << "  ESC   - 退出" << std::endl;
        std::cout << "  R     - 重置已采集的图像" << std::endl;
        std::cout << "======================\n" << std::endl;

        const std::string window_name = "相机标定 - 实时采集";
        cv::namedWindow(window_name, cv::WINDOW_NORMAL);

        while (true) {
            cv::Mat frame;
            if (!camera->grabFrame(frame)) {
                continue;
            }

            cv::Mat display = frame.clone();

            // 检测当前帧中的棋盘格
            std::vector<cv::Point2f> corners;
            bool found = detectChessboard(frame, corners);

            if (found) {
                drawDetectedCorners(display, corners, config_.getBoardSize(), true);
            }

            // 显示已采集数量
            std::string info = "已采集: " + std::to_string(captured_images_.size())
                             + "/" + std::to_string(config_.min_images_required)
                             + " (最少要求)";
            cv::putText(display, info, cv::Point(10, 30),
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);

            if (found) {
                cv::putText(display, "检测到棋盘格 - 按 SPACE 保存",
                            cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX,
                            0.7, cv::Scalar(0, 255, 0), 2);
            } else {
                cv::putText(display, "未检测到棋盘格 - 调整位姿",
                            cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX,
                            0.7, cv::Scalar(0, 0, 255), 2);
            }

            cv::imshow(window_name, display);
            int key = cv::waitKey(30) & 0xFF;

            if (key == 27) {  // ESC
                std::cout << "用户退出采集" << std::endl;
                break;
            } else if (key == ' ' || key == 32) {  // SPACE
                if (found) {
                    captured_images_.push_back(frame.clone());
                    std::cout << "[保存] 第 " << captured_images_.size()
                              << " 张图像已保存" << std::endl;
                } else {
                    std::cout << "[跳过] 当前帧未检测到棋盘格" << std::endl;
                }
            } else if (key == 'r' || key == 'R') {
                captured_images_.clear();
                std::cout << "[重置] 已清除所有采集的图像" << std::endl;
            } else if (key == 13) {  // ENTER
                if (captured_images_.size() >= static_cast<size_t>(config_.min_images_required)) {
                    std::cout << "采集完成，开始标定..." << std::endl;
                    break;
                } else {
                    std::cout << "[提示] 有效图像不足 " << config_.min_images_required
                              << " 张，继续采集 (当前: " << captured_images_.size() << ")"
                              << std::endl;
                }
            }
        }

        cv::destroyWindow(window_name);
    }

    // 将采集的图像保存到文件并转换为文件路径列表
    std::vector<std::string> image_paths;
    if (ensureDirectory(config_.output_dir + "/captured_images")) {
        for (size_t i = 0; i < captured_images_.size(); ++i) {
            std::string path = config_.output_dir + "/captured_images/frame_" +
                               std::to_string(i) + ".png";
            cv::imwrite(path, captured_images_[i]);
            image_paths.push_back(path);
        }
    }

    // 委托给文件标定流程
    return calibrateFromFiles(image_paths);
}

// ============================================================================
// 结果持久化
// ============================================================================

CalibrationStatus CameraCalibration::saveCalibration(
    const CalibrationResult& result,
    const std::string& filepath)
{
    try {
        cv::FileStorage fs(filepath, cv::FileStorage::WRITE);

        if (!fs.isOpened()) {
            setError(CalibrationStatus::ERROR_FILE_IO,
                     "无法打开文件写入: " + filepath);
            return CalibrationStatus::ERROR_FILE_IO;
        }

        // 写入标定元数据
        fs << "calibration_version" << "M1_v1.0.0";
        fs << "calibration_timestamp" << result.timestamp;
        fs << "camera_model" << "pinhole";

        // 写入相机内参
        fs << "camera_matrix" << result.camera_matrix;

        // 写入畸变系数
        fs << "distortion_coeffs" << result.distortion_coeffs;

        // 写入畸变模型描述
        std::string dist_model;
        int num_coeffs = result.distortion_coeffs.total();
        if (num_coeffs == 5) {
            dist_model = "plumb_bob (k1,k2,p1,p2,k3)";
        } else if (num_coeffs == 8) {
            dist_model = "rational (k1-k6,p1,p2)";
        } else {
            dist_model = "unknown (" + std::to_string(num_coeffs) + " coeffs)";
        }
        fs << "distortion_model" << dist_model;

        // 写入图像尺寸
        fs << "image_width" << result.image_size.width;
        fs << "image_height" << result.image_size.height;

        // 写入标定质量
        fs << "reprojection_error" << result.reprojection_error;
        fs << "images_used" << result.images_used;
        fs << "images_total" << result.images_total;
        fs << "calibration_time_ms" << result.calibration_time_ms;

        // 写入标定配置
        fs << "calibration_config" << "{";
        fs << "board_width" << result.config.board_width;
        fs << "board_height" << result.config.board_height;
        fs << "square_size_mm" << result.config.square_size_mm;
        fs << "use_rational_model" << (result.config.use_rational_model ? 1 : 0);
        fs << "}";

        // 写入每张图像的误差
        fs << "per_view_errors" << "[";
        for (size_t i = 0; i < result.per_view_errors.size(); ++i) {
            fs << result.per_view_errors[i];
        }
        fs << "]";

        // 写入重投影误差统计
        fs << "error_statistics" << "{";
        fs << "mean" << result.computeMeanError();
        fs << "max" << result.computeMaxError();
        fs << "std" << result.computeStdError();
        fs << "}";

        // 额外保存为 TXT 格式（方便 C# 直接读取）
        std::string txt_path = filepath.substr(0, filepath.find_last_of('.')) + ".txt";
        std::ofstream txt_file(txt_path);
        if (txt_file.is_open()) {
            txt_file << "=== 相机标定结果 ===\n";
            txt_file << "时间: " << result.timestamp << "\n";
            txt_file << "重投影误差: " << result.reprojection_error << " px\n";
            txt_file << "图像尺寸: " << result.image_size.width << " x "
                     << result.image_size.height << "\n\n";
            txt_file << "内参矩阵 (Camera Matrix):\n";
            txt_file << result.camera_matrix << "\n\n";
            txt_file << "畸变系数 (Distortion Coefficients):\n";
            txt_file << result.distortion_coeffs << "\n\n";
            txt_file << "畸变模型: " << dist_model << "\n";
            txt_file << "使用图像数: " << result.images_used << " / "
                     << result.images_total << "\n";
            txt_file << "每张图像误差:\n";
            for (size_t i = 0; i < result.per_view_errors.size(); ++i) {
                txt_file << "  图像 " << (i + 1) << ": "
                         << result.per_view_errors[i] << " px\n";
            }
            txt_file << "\n误差统计:\n";
            txt_file << "  均值: " << result.computeMeanError() << " px\n";
            txt_file << "  最大值: " << result.computeMaxError() << " px\n";
            txt_file << "  标准差: " << result.computeStdError() << " px\n";
            txt_file.close();
        }

        fs.release();

        std::cout << "标定结果已保存至: " << filepath << std::endl;
        std::cout << "TXT 格式已保存至: " << txt_path << std::endl;

        setError(CalibrationStatus::SUCCESS, "保存成功");
        return CalibrationStatus::SUCCESS;

    } catch (const cv::Exception& e) {
        setError(CalibrationStatus::ERROR_FILE_IO,
                 "OpenCV 异常: " + std::string(e.what()));
        return CalibrationStatus::ERROR_FILE_IO;
    } catch (const std::exception& e) {
        setError(CalibrationStatus::ERROR_FILE_IO,
                 "标准异常: " + std::string(e.what()));
        return CalibrationStatus::ERROR_FILE_IO;
    }
}

CalibrationStatus CameraCalibration::loadCalibration(
    const std::string& filepath,
    CalibrationResult& result)
{
    try {
        cv::FileStorage fs(filepath, cv::FileStorage::READ);

        if (!fs.isOpened()) {
            setError(CalibrationStatus::ERROR_FILE_IO,
                     "无法打开文件读取: " + filepath);
            return CalibrationStatus::ERROR_FILE_IO;
        }

        // 读取标定信息
        fs["calibration_timestamp"] >> result.timestamp;
        fs["camera_matrix"] >> result.camera_matrix;
        fs["distortion_coeffs"] >> result.distortion_coeffs;
        fs["image_width"] >> result.image_size.width;
        fs["image_height"] >> result.image_size.height;
        fs["reprojection_error"] >> result.reprojection_error;
        fs["images_used"] >> result.images_used;

        // 读取配置
        cv::FileNode config_node = fs["calibration_config"];
        if (!config_node.empty()) {
            config_node["board_width"] >> result.config.board_width;
            config_node["board_height"] >> result.config.board_height;
            config_node["square_size_mm"] >> result.config.square_size_mm;
        }

        // 读取每张图像误差
        cv::FileNode errors_node = fs["per_view_errors"];
        if (!errors_node.empty()) {
            result.per_view_errors.clear();
            for (auto& e : errors_node) {
                double val;
                e >> val;
                result.per_view_errors.push_back(val);
            }
        }

        fs.release();

        result.config = config_;
        result.timestamp = getCurrentTimestamp();

        setError(CalibrationStatus::SUCCESS, "加载成功");
        return CalibrationStatus::SUCCESS;

    } catch (const cv::Exception& e) {
        setError(CalibrationStatus::ERROR_FILE_IO,
                 "OpenCV 异常: " + std::string(e.what()));
        return CalibrationStatus::ERROR_FILE_IO;
    } catch (const std::exception& e) {
        setError(CalibrationStatus::ERROR_FILE_IO,
                 "标准异常: " + std::string(e.what()));
        return CalibrationStatus::ERROR_FILE_IO;
    }
}

// ============================================================================
// 去畸变校正
// ============================================================================

UndistortMaps CameraCalibration::computeUndistortMaps(
    const cv::Mat& camera_matrix,
    const cv::Mat& dist_coeffs,
    const cv::Size& image_size,
    double alpha)
{
    UndistortMaps maps;

    if (camera_matrix.empty() || dist_coeffs.empty()) {
        setError(CalibrationStatus::ERROR_NOT_CALIBRATED,
                 "请先完成标定，或加载标定结果");
        return maps;
    }

    // 计算新的内参矩阵（考虑 alpha 参数）
    cv::Mat new_camera_matrix = cv::getOptimalNewCameraMatrix(
        camera_matrix, dist_coeffs, image_size, alpha, image_size, nullptr);

    // 预计算去畸变映射（用于 cv::remap）
    // 映射类型：CV_16SC2 比 CV_32FC1 更快（整数插值）
    cv::initUndistortRectifyMap(
        camera_matrix,      // 原始内参
        dist_coeffs,        // 畸变系数
        cv::Mat(),          // 旋转矩阵（无旋转）
        new_camera_matrix,  // 新内参（校正后）
        image_size,         // 图像尺寸
        CV_16SC2,           // 映射类型（16位有符号，性能最优）
        maps.map1,          // 输出 x 映射
        maps.map2);         // 输出 y 映射

    // 保存新内参矩阵（用于后续 3D 重建）
    // 注意：校正后的内参矩阵和原始内参不同，3D 重建时要使用新内参
    (void)new_camera_matrix;  // 如果需要，可以添加到 UndistortMaps 结构体中

    return maps;
}

bool CameraCalibration::undistortImage(
    const cv::Mat& distorted,
    cv::Mat& undistorted,
    const UndistortMaps& maps)
{
    if (!maps.valid()) {
        setError(CalibrationStatus::ERROR_NOT_CALIBRATED,
                 "去畸变映射无效，请先调用 computeUndistortMaps");
        return false;
    }

    // cv::remap 是最高效的去畸变方式（比 cv::undistort 快 2-3 倍）
    cv::remap(distorted, undistorted, maps.map1, maps.map2,
              cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));

    return true;
}

bool CameraCalibration::undistortImageDirect(
    const cv::Mat& distorted,
    cv::Mat& undistorted,
    const cv::Mat& camera_matrix,
    const cv::Mat& dist_coeffs)
{
    if (distorted.empty() || camera_matrix.empty() || dist_coeffs.empty()) {
        return false;
    }

    // 便捷方法：直接使用 cv::undistort（适合一次性的校正操作）
    cv::undistort(distorted, undistorted, camera_matrix, dist_coeffs);
    return true;
}

// ============================================================================
// 质量评估与可视化
// ============================================================================

std::string CameraCalibration::generateReport(const CalibrationResult& result) {
    std::stringstream ss;

    ss << "========================================\n";
    ss << "  相机标定质量报告\n";
    ss << "========================================\n\n";

    ss << "【基本信息】\n";
    ss << "  标定时间: " << result.timestamp << "\n";
    ss << "  使用图像: " << result.images_used << " / " << result.images_total << "\n";
    ss << "  标定耗时: " << std::fixed << std::setprecision(1)
       << result.calibration_time_ms << " ms\n\n";

    ss << "【标定质量】\n";
    ss << "  重投影误差: " << std::fixed << std::setprecision(4)
       << result.reprojection_error << " px\n";
    ss << "  误差均值:   " << result.computeMeanError() << " px\n";
    ss << "  误差最大值: " << result.computeMaxError() << " px\n";
    ss << "  误差标准差: " << result.computeStdError() << " px\n";

    // 质量评级
    ss << "  质量评级:   ";
    if (result.reprojection_error < 0.15) {
        ss << "优秀 (Excellent)";
    } else if (result.reprojection_error < 0.30) {
        ss << "良好 (Good)";
    } else if (result.reprojection_error < 0.50) {
        ss << "可接受 (Acceptable)";
    } else {
        ss << "需要重新标定 (Poor)";
    }
    ss << "\n\n";

    ss << "【相机内参矩阵】\n";
    ss << "  fx = " << result.camera_matrix.at<double>(0, 0) << "\n";
    ss << "  fy = " << result.camera_matrix.at<double>(1, 1) << "\n";
    ss << "  cx = " << result.camera_matrix.at<double>(0, 2) << "\n";
    ss << "  cy = " << result.camera_matrix.at<double>(1, 2) << "\n\n";

    ss << "【畸变系数】\n";
    for (int i = 0; i < result.distortion_coeffs.total(); ++i) {
        ss << "  k" << (i + 1) << " = " << std::scientific
           << result.distortion_coeffs.at<double>(0, i) << "\n";
    }
    ss << "\n";

    // 误差分布
    ss << "【误差分布】\n";
    for (size_t i = 0; i < result.per_view_errors.size(); ++i) {
        std::string bar;
        int bar_len = static_cast<int>(result.per_view_errors[i] * 50);
        if (bar_len > 50) bar_len = 50;
        bar.append(bar_len, '#');
        ss << "  图像 " << std::setw(2) << (i + 1) << ": "
           << std::fixed << std::setprecision(3)
           << result.per_view_errors[i] << " px  " << bar << "\n";
    }
    ss << "\n";

    ss << "【建议】\n";
    if (result.reprojection_error < 0.15) {
        ss << "  标定质量优秀，可以用于焊接定位。\n";
    } else if (result.reprojection_error < 0.30) {
        ss << "  标定质量良好，满足 ±0.5mm 焊接定位要求。\n";
    } else if (result.reprojection_error < 0.50) {
        ss << "  标定质量可接受，建议检查并排除误差较大的图像后重新标定。\n";
    } else {
        ss << "  标定误差过大，建议：\n";
        ss << "    1. 检查棋盘格打印质量（是否平整、反光）\n";
        ss << "    2. 增加图像数量和位姿变化\n";
        ss << "    3. 确保图像清晰（避免运动模糊）\n";
        ss << "    4. 检查标定板方格尺寸是否正确\n";
    }
    ss << "========================================\n";

    return ss.str();
}

cv::Mat CameraCalibration::visualizeCalibrationQuality(
    const CalibrationResult& result)
{
    if (result.per_view_errors.empty()) {
        cv::Mat blank(300, 400, CV_8UC3, cv::Scalar(255, 255, 255));
        cv::putText(blank, "No data available", cv::Point(50, 150),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 0), 2);
        return blank;
    }

    // 创建误差分布直方图
    int num_images = static_cast<int>(result.per_view_errors.size());
    int width = std::max(800, num_images * 30);
    int height = 500;
    cv::Mat hist(height, width, CV_8UC3, cv::Scalar(255, 255, 255));

    // 找最大值用于归一化
    double max_error = *std::max_element(
        result.per_view_errors.begin(), result.per_view_errors.end());

    // 绘制基线
    int margin = 60;
    int baseline = height - margin;
    cv::line(hist, cv::Point(margin, baseline),
             cv::Point(width - margin, baseline), cv::Scalar(0, 0, 0), 2);

    // 绘制每张图像的误差柱状图
    int bar_width = (width - 2 * margin) / num_images;

    for (int i = 0; i < num_images; ++i) {
        int bar_height = static_cast<int>(
            (result.per_view_errors[i] / max_error) * (baseline - margin - 10));

        int x = margin + i * bar_width + 2;
        int y = baseline - bar_height;

        // 颜色：绿色（好） -> 黄色（中） -> 红色（差）
        cv::Scalar color;
        double ratio = result.per_view_errors[i] / (max_error + 1e-6);
        if (ratio < 0.33) {
            color = cv::Scalar(0, 200, 0);   // 绿色
        } else if (ratio < 0.66) {
            color = cv::Scalar(0, 200, 200); // 黄色
        } else {
            color = cv::Scalar(0, 0, 200);   // 红色
        }

        cv::rectangle(hist, cv::Rect(x, y, bar_width - 4, bar_height),
                      color, cv::FILLED);

        // 标注数值
        std::string text = std::to_string(result.per_view_errors[i]).substr(0, 4);
        cv::putText(hist, text, cv::Point(x, y - 5),
                    cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(0, 0, 0), 1);
    }

    // 标题和标签
    cv::putText(hist, "Reprojection Error per Image",
                cv::Point(width / 2 - 150, 30),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 0), 2);

    cv::putText(hist, "Image Index", cv::Point(width / 2 - 40, height - 10),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);

    // 旋转 Y 轴标签
    cv::putText(hist, "Error (px)", cv::Point(10, height / 2),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);

    // 显示总体误差
    std::string summary = "Overall Error: " +
        std::to_string(result.reprojection_error).substr(0, 5) + " px";
    cv::putText(hist, summary, cv::Point(width - 250, 30),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(200, 0, 0), 2);

    return hist;
}

cv::Mat CameraCalibration::visualizeUndistortion(
    const cv::Mat& original,
    const UndistortMaps& maps)
{
    if (original.empty() || !maps.valid()) {
        return cv::Mat();
    }

    cv::Mat undistorted;
    undistortImage(original, undistorted, maps);

    // 左右拼接：原始图像 | 校正后图像
    int total_width = original.cols * 2;
    int height = original.rows;

    cv::Mat comparison(height, total_width, original.type());
    original.copyTo(comparison(cv::Rect(0, 0, original.cols, original.rows)));
    undistorted.copyTo(comparison(cv::Rect(original.cols, 0, original.cols, original.rows)));

    // 添加标签
    cv::putText(comparison, "Original (Distorted)", cv::Point(10, 30),
                cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
    cv::putText(comparison, "Undistorted", cv::Point(original.cols + 10, 30),
                cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);

    // 画一条分割线
    cv::line(comparison, cv::Point(original.cols, 0),
             cv::Point(original.cols, height), cv::Scalar(0, 0, 255), 2);

    return comparison;
}

// ============================================================================
// 批量检测
// ============================================================================

int CameraCalibration::batchDetectFromDirectory(
    const std::string& input_dir,
    const std::string& output_dir,
    std::vector<ImageDetectionResult>& corner_output)
{
    corner_output.clear();

    if (!ensureDirectory(output_dir)) {
        return 0;
    }

    // 收集所有图像文件
    std::vector<std::string> image_files;
    for (const auto& entry : std::filesystem::directory_iterator(input_dir)) {
        if (!entry.is_regular_file()) continue;

        std::string ext = entry.path().extension().string();
        std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);

        if (ext == ".jpg" || ext == ".jpeg" || ext == ".png" ||
            ext == ".bmp" || ext == ".tiff" || ext == ".tif") {
            image_files.push_back(entry.path().string());
        }
    }

    std::sort(image_files.begin(), image_files.end());

    std::cout << "批量检测: " << image_files.size() << " 张图像" << std::endl;

    int success_count = 0;
    for (const auto& img_path : image_files) {
        ImageDetectionResult det_result;
        det_result.image_path = img_path;

        auto start = std::chrono::high_resolution_clock::now();

        cv::Mat image = cv::imread(img_path, cv::IMREAD_COLOR);
        if (image.empty()) {
            det_result.success = false;
            corner_output.push_back(det_result);
            continue;
        }

        std::vector<cv::Point2f> corners;
        bool found = detectChessboard(image, corners);

        auto end = std::chrono::high_resolution_clock::now();
        det_result.detection_time_ms = std::chrono::duration<double, std::milli>(
            end - start).count();

        if (found) {
            det_result.success = true;
            det_result.corners = corners;
            det_result.image_size = image.size();
            success_count++;

            // 保存标注了角点的图像
            cv::Mat display = image.clone();
            drawDetectedCorners(display, corners, config_.getBoardSize(), true);

            std::string out_path = output_dir + "/" +
                std::filesystem::path(img_path).stem().string() + "_corners.png";
            cv::imwrite(out_path, display);
        }

        corner_output.push_back(det_result);
    }

    std::cout << "批量检测完成: " << success_count << "/"
              << image_files.size() << " 成功" << std::endl;

    return success_count;
}

} // namespace bus_welding