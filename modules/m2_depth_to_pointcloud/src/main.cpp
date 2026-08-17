/**
 * @file main.cpp
 * @brief M2 深度图→点云生成 - 命令行示例程序
 *
 * 使用示例：
 *   # 从深度图文件生成点云
 *   M2_DepthToPointCloud.exe -d depth.png -K camera_matrix.yml -o output.pcd
 *
 *   # 从深度图+彩色图生成彩色点云
 *   M2_DepthToPointCloud.exe -d depth.png -c color.png -K camera_matrix.yml -o output.ply
 *
 *   # 批量处理目录中所有深度图
 *   M2_DepthToPointCloud.exe -b ./depth_images/ -K camera_matrix.yml -o ./output/
 *
 *   # 可视化深度图
 *   M2_DepthToPointCloud.exe -v depth.png -K camera_matrix.yml
 *
 *   # 查看点云统计信息
 *   M2_DepthToPointCloud.exe -s pointcloud.pcd
 */

#include "depth_to_pointcloud.hpp"
#include <iostream>
#include <filesystem>
#include <cstring>

using namespace bus_welding;

void printHelp(const char* program_name) {
    std::cout << "\n========================================\n";
    std::cout << "  M2 深度图→点云生成模块 - 使用说明\n";
    std::cout << "========================================\n\n";
    std::cout << "用法:\n";
    std::cout << "  " << program_name << " -d <depth> -K <calib> [-c <color>] [-o <output>]\n\n";
    std::cout << "参数:\n";
    std::cout << "  -d <depth>      深度图文件 (16位PNG或32位TIFF)\n";
    std::cout << "  -K <calib>      相机内参文件 (YAML格式，来自M1)\n";
    std::cout << "  -c <color>      彩色图文件 (可选，生成彩色点云)\n";
    std::cout << "  -o <output>     输出点云文件 (.pcd 或 .ply)\n";
    std::cout << "  -b <dir> <out>  批量处理目录\n";
    std::cout << "  -v <depth>      可视化深度图\n";
    std::cout << "  -s <pcd>        查看点云统计\n";
    std::cout << "  -f <format>     深度格式: mm(默认), cm, m, raw16, raw32\n";
    std::cout << "  --organized     输出组织化点云 (默认)\n";
    std::cout << "  --unorganized   输出非组织化点云\n";
    std::cout << "  -h              显示此帮助\n\n";
    std::cout << "示例:\n";
    std::cout << "  " << program_name << " -d depth.png -K calib.yml -o cloud.pcd\n";
    std::cout << "  " << program_name << " -d depth.png -c color.png -K calib.yml -o cloud.ply\n";
    std::cout << "  " << program_name << " -v depth.png -K calib.yml\n";
    std::cout << "========================================\n" << std::endl;
}

/// 加载相机内参
bool loadCameraMatrix(const std::string& calib_file, cv::Mat& camera_matrix) {
    cv::FileStorage fs(calib_file, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "[ERROR] 无法加载标定文件: " << calib_file << std::endl;
        return false;
    }
    fs["camera_matrix"] >> camera_matrix;
    fs.release();

    if (camera_matrix.empty()) {
        std::cerr << "[ERROR] 标定文件中未找到 camera_matrix" << std::endl;
        return false;
    }

    std::cout << "[M2] 相机内参加载成功:\n" << camera_matrix << std::endl;
    return true;
}

/// 解析深度格式
DepthToPointCloud::DepthFormat parseDepthFormat(const std::string& fmt) {
    if (fmt == "mm")    return DepthToPointCloud::DepthFormat::CV_16UC1_MM;
    if (fmt == "cm")    return DepthToPointCloud::DepthFormat::CV_16UC1_CM;
    if (fmt == "m")     return DepthToPointCloud::DepthFormat::CV_32FC1_M;
    if (fmt == "raw16") return DepthToPointCloud::DepthFormat::CV_16UC1_RAW;
    if (fmt == "raw32") return DepthToPointCloud::DepthFormat::CV_32FC1_RAW;
    return DepthToPointCloud::DepthFormat::CV_16UC1_MM;
}

/// 单张深度图→点云
int runSingleConversion(
    const std::string& depth_file,
    const std::string& color_file,
    const std::string& calib_file,
    const std::string& output_file,
    DepthToPointCloud::DepthFormat depth_fmt,
    bool organized)
{
    // 加载相机内参
    cv::Mat camera_matrix;
    if (!loadCameraMatrix(calib_file, camera_matrix)) return -1;

    // 加载深度图
    cv::Mat depth = cv::imread(depth_file, cv::IMREAD_UNCHANGED);
    if (depth.empty()) {
        std::cerr << "[ERROR] 无法读取深度图: " << depth_file << std::endl;
        return -1;
    }

    std::cout << "[M2] 深度图加载成功: " << depth_file
              << " (" << depth.cols << "x" << depth.rows
              << ", type=" << depth.type() << ")" << std::endl;

    // 配置
    DepthToPointCloud::Config config;
    config.depth_format = depth_fmt;
    config.organized = organized;

    DepthToPointCloud converter(config);

    // 转换
    if (!color_file.empty()) {
        // 彩色点云
        cv::Mat color = cv::imread(color_file, cv::IMREAD_COLOR);
        if (color.empty()) {
            std::cerr << "[ERROR] 无法读取彩色图: " << color_file << std::endl;
            return -1;
        }

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        auto start = std::chrono::high_resolution_clock::now();
        bool ok = converter.depthToColoredPointCloud(depth, color, camera_matrix, cloud);
        auto end = std::chrono::high_resolution_clock::now();

        if (!ok) {
            std::cerr << "[ERROR] 彩色点云转换失败" << std::endl;
            return -1;
        }

        double ms = std::chrono::duration<double, std::milli>(end - start).count();
        std::cout << "[M2] 彩色点云转换完成: " << cloud->size() << " 点, 耗时 " << ms << " ms" << std::endl;

        if (!output_file.empty()) {
            converter.saveColoredPointCloud(output_file, cloud);
        }
    } else {
        // XYZ点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        auto start = std::chrono::high_resolution_clock::now();
        bool ok = converter.depthToPointCloud(depth, camera_matrix, cloud);
        auto end = std::chrono::high_resolution_clock::now();

        if (!ok) {
            std::cerr << "[ERROR] 点云转换失败" << std::endl;
            return -1;
        }

        double ms = std::chrono::duration<double, std::milli>(end - start).count();
        std::cout << "[M2] 点云转换完成: " << cloud->size() << " 点, 耗时 " << ms << " ms" << std::endl;

        // 统计信息
        auto stats = converter.computeCloudStats(cloud);
        std::cout << converter.generateStatsReport(stats) << std::endl;

        if (!output_file.empty()) {
            converter.savePointCloud(output_file, cloud);
        }
    }

    return 0;
}

/// 可视化深度图
int runVisualize(const std::string& depth_file, const std::string& calib_file) {
    cv::Mat depth = cv::imread(depth_file, cv::IMREAD_UNCHANGED);
    if (depth.empty()) return -1;

    // 加载相机内参（用于显示图像尺寸信息）
    if (!calib_file.empty()) {
        cv::Mat K;
        loadCameraMatrix(calib_file, K);
    }

    DepthToPointCloud converter;
    cv::Mat depth_color = converter.renderDepthColorMap(depth);

    if (!depth_color.empty()) {
        cv::imshow("Depth Visualization", depth_color);
        cv::imshow("Original Depth", depth);

        std::cout << "按任意键退出..." << std::endl;
        cv::waitKey(0);
        cv::destroyAllWindows();
    }

    return 0;
}

/// 查看点云统计
int runStats(const std::string& pcd_file) {
    DepthToPointCloud converter;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    if (!converter.loadPointCloud(pcd_file, cloud)) return -1;

    auto stats = converter.computeCloudStats(cloud);
    std::cout << converter.generateStatsReport(stats) << std::endl;

    return 0;
}

/// 批量处理
int runBatch(
    const std::string& input_dir,
    const std::string& calib_file,
    const std::string& output_dir,
    DepthToPointCloud::DepthFormat depth_fmt)
{
    cv::Mat camera_matrix;
    if (!loadCameraMatrix(calib_file, camera_matrix)) return -1;

    std::filesystem::create_directories(output_dir);

    DepthToPointCloud::Config config;
    config.depth_format = depth_fmt;
    DepthToPointCloud converter(config);

    int success = 0;
    int total = 0;

    for (const auto& entry : std::filesystem::directory_iterator(input_dir)) {
        if (!entry.is_regular_file()) continue;

        std::string ext = entry.path().extension().string();
        std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);

        if (ext == ".png" || ext == ".tiff" || ext == ".tif") {
            total++;
            std::string depth_path = entry.path().string();
            std::string out_name = entry.path().stem().string() + ".pcd";
            std::string out_path = output_dir + "/" + out_name;

            std::cout << "[" << total << "] 处理: " << depth_path << std::endl;

            cv::Mat depth = cv::imread(depth_path, cv::IMREAD_UNCHANGED);
            if (depth.empty()) continue;

            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
            if (converter.depthToPointCloud(depth, camera_matrix, cloud)) {
                converter.savePointCloud(out_path, cloud);
                success++;
            }
        }
    }

    std::cout << "\n批量处理完成: " << success << "/" << total << " 成功" << std::endl;
    return 0;
}

int main(int argc, char** argv) {
    std::cout << "========================================\n";
    std::cout << "  M2 深度图→点云生成模块 v1.0.0\n";
    std::cout << "  客车焊接 3D 视觉引导系统\n";
    std::cout << "========================================\n" << std::endl;

    if (argc < 2) {
        printHelp(argv[0]);
        return 0;
    }

    std::string depth_file, color_file, calib_file, output_file, input_dir;
    std::string depth_fmt_str = "mm";
    bool organized = true;

    // 解析参数
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "-h" || arg == "--help") {
            printHelp(argv[0]);
            return 0;
        } else if (arg == "-d" && i + 1 < argc) {
            depth_file = argv[++i];
        } else if (arg == "-c" && i + 1 < argc) {
            color_file = argv[++i];
        } else if (arg == "-K" && i + 1 < argc) {
            calib_file = argv[++i];
        } else if (arg == "-o" && i + 1 < argc) {
            output_file = argv[++i];
        } else if (arg == "-b" && i + 2 < argc) {
            input_dir = argv[++i];
            output_file = argv[++i];
        } else if (arg == "-v" && i + 1 < argc) {
            depth_file = argv[++i];
            if (i + 1 < argc && std::string(argv[i+1])[0] != '-') {
                calib_file = argv[++i];
            }
            return runVisualize(depth_file, calib_file);
        } else if (arg == "-s" && i + 1 < argc) {
            return runStats(argv[++i]);
        } else if (arg == "-f" && i + 1 < argc) {
            depth_fmt_str = argv[++i];
        } else if (arg == "--organized") {
            organized = true;
        } else if (arg == "--unorganized") {
            organized = false;
        }
    }

    DepthToPointCloud::DepthFormat depth_fmt = parseDepthFormat(depth_fmt_str);

    try {
        if (!input_dir.empty()) {
            return runBatch(input_dir, calib_file, output_file, depth_fmt);
        } else if (!depth_file.empty()) {
            return runSingleConversion(
                depth_file, color_file, calib_file, output_file,
                depth_fmt, organized);
        } else {
            printHelp(argv[0]);
            return -1;
        }
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] 异常: " << e.what() << std::endl;
        return -1;
    }
}
