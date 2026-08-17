# 客车焊接3D视觉引导系统

> Bus Welding 3D Vision Guide System  
> 基于 3D 相机 + 六轴机器人的客车车身智能焊接引导系统

---

## 项目概述

本项目实现了一套完整的**客车车身智能焊接 3D 视觉引导系统**，涵盖从相机标定、点云生成、焊缝提取到路径规划的全流程算法模块。

### 核心目标

| 指标 | 要求 |
|------|------|
| 焊接定位精度 | ±0.5mm |
| 单条焊缝编程时间 | ≤1分钟 |
| 视觉引导方式 | 3D Eye-in-Hand |
| 检测对象 | 客车车身焊缝 |

### 硬件方案

- **3D 相机**: Eye-in-Hand 安装方式（相机固定在机器人末端）
- **机器人**: 六轴工业机器人
- **焊接系统**: 弧焊系统

---

## 相机选型

### 最终选择：海康威视 (Hikvision) 工业相机

| 对比项 | 海康威视 Hikvision | Basler | 大恒 Daheng |
|--------|-------------------|--------|-------------|
| SDK 完善度 | 优秀（MVS SDK，文档齐全） | 优秀（Pylon SDK） | 良好 |
| 3D 相机支持 | 支持双目/结构光/ToF | 以2D为主 | 有3D产品线 |
| 机器人集成 | 案例丰富，兼容性好 | 一般 | 较少 |
| 性价比 | 高 | 中 | 高 |
| 国内技术支持 | 本地化支持完善 | 代理商支持 | 本地化支持 |

**选择理由**:
1. MVS SDK 提供完善的 C++ API，与本项目技术栈完全兼容
2. 在国内客车焊接领域有已落地案例，焊縫定位效果经过验证
3. 与六轴机器人系统的集成案例丰富
4. 相机型号选择灵活（从 GigE 到 USB3.0 接口）

**推荐型号**:
- 3D 结构光相机: Hikrobot MV-DL1300-04H 系列
- 双目立体相机: Hikrobot MV-DB1300-04H 系列
- 备选: 可搭配任何支持 OpenCV 的工业相机（通过 CameraInterface 抽象接口适配）

---

## 项目架构

### 五层架构设计

`
┌─────────────────────────────────────────────────────────┐
│  第5层: 界面层 (WPF .NET 6/8)                          │
│  - 相机控制面板 / 标定界面 / 点云可视化 / 焊缝显示     │
├─────────────────────────────────────────────────────────┤
│  第4层: 系统状态机 (C# 状态管理)                       │
│  - 空闲 → 标定 → 扫描 → 提取 → 规划 → 焊接 → 纠偏    │
├─────────────────────────────────────────────────────────┤
│  第3层: 算法层 (C++/OpenCV/PCL)                        │
│  M1 标定 │ M2 点云 │ M3 滤波 │ M4 焊缝 │ M5 手眼标定  │
├─────────────────────────────────────────────────────────┤
│  第2层: 数据层 (YAML/JSON/PCD)                         │
│  标定参数 / 点云文件 / 焊缝路径 / 机器人位姿           │
├─────────────────────────────────────────────────────────┤
│  第1层: 设备层 (相机SDK / 机器人接口 / PLC)            │
│  海康SDK / 机器人API / HslCommunication                │
└─────────────────────────────────────────────────────────┘
`

### 核心模块

| 模块 | 状态 | 功能 | 技术 |
|------|------|------|------|
| **M1** | ✅ 已完成 | 相机内参标定（含畸变校正） | OpenCV calibrateCamera |
| **M2** | ✅ 已完成 | 深度图→点云生成 | PCL + 反向投影 |
| M3 | ❌ 待开发 | 点云滤波（降采样/去噪） | PCL filters |
| M4 | ❌ 待开发 | 焊缝特征提取与路径规划 | PCL + 自定义算法 |
| M5 | ❌ 待开发 | 手眼标定 | OpenCV + 机器人位姿 |
| M6 | ❌ 待开发 | C++ DLL 封装 + WPF 集成 | P/Invoke / C++/CLI |

---

## 技术栈

### 开发环境

| 工具 | 版本 | 用途 |
|------|------|------|
| Visual Studio | 2022 | 主 IDE |
| CMake | ≥ 3.20 | 构建系统 |
| vcpkg | 2024+ | 包管理器 |
| Git | - | 版本控制 |

### 核心依赖

| 库 | 版本 | 模块 | 用途 |
|----|------|------|------|
| OpenCV | 4.8+ | M1, M2, M5 | 相机标定、图像处理、矩阵运算 |
| PCL | 1.14+ | M2, M3, M4 | 点云处理、滤波、分割 |

### 编程语言

- **C++17** — 核心算法模块（高性能、精确控制）
- **C# / .NET 6/8** — WPF 上位机界面（快速开发、UI丰富）
- **C++/CLI** — 桥接层（可选，用于 C++/C# 互操作）

---

## 快速开始

### 1. 安装依赖

`powershell
# 安装 vcpkg（如未安装）
git clone https://github.com/microsoft/vcpkg.git
cd vcpkg
.\bootstrap-vcpkg.bat
.\vcpkg integrate install

# 安装 OpenCV
.\vcpkg install opencv4[core,calib3d,imgproc,imgcodecs,highgui]:x64-windows

# 安装 PCL
.\vcpkg install pcl[common,io,filters]:x64-windows
`

### 2. 构建项目

`powershell
# 克隆仓库
git clone https://github.com/grapefruit3c/3D-Weld-Seam-Tracking-using-PCL.git
cd 3D-Weld-Seam-Tracking-using-PCL

# 配置 CMake
cmake -B build -S . 
    -DCMAKE_TOOLCHAIN_FILE=<vcpkg_root>/scripts/buildsystems/vcpkg.cmake 
    -DCMAKE_BUILD_TYPE=Release

# 编译
cmake --build build --config Release
`

### 3. 运行 M1 相机标定

`powershell
# 从图像文件标定
./build/bin/M1_CameraCalibration.exe -f ./calib_images -o ./result

# 从相机标定（交互模式，空格保存，回车标定）
./build/bin/M1_CameraCalibration.exe -c 0 -o ./result

# 查看标定报告
./build/bin/M1_CameraCalibration.exe -s ./result/calibration_result.yml
`

### 4. 运行 M2 深度图→点云

`powershell
# 单张深度图转换
./build/bin/M2_DepthToPointCloud.exe -d depth.png -K calib.yml -o cloud.pcd

# 带彩色图的彩色点云
./build/bin/M2_DepthToPointCloud.exe -d depth.png -c color.png -K calib.yml -o cloud.ply

# 批量处理
./build/bin/M2_DepthToPointCloud.exe -b ./depth_images/ -K calib.yml -o ./output/
`

---

## 标定图像采集指南

### 棋盘格标定板

`
┌─────────────────────────────────────┐
│  棋盘格参数: 9×6 内角点             │
│  方格尺寸: 30mm（推荐）              │
│  生成工具: calib.io 或 OpenCV 打印  │
└─────────────────────────────────────┘
`

### 采集要求（15-25张）

| 位姿类型 | 说明 | 数量 |
|----------|------|------|
| 正对 | 标定板正对相机 | 3-5张 |
| 五区域 | 中央/左上/右上/左下/右下 | 5-8张 |
| 倾斜 | 倾斜 ±15°~45° | 5-8张 |
| 旋转 | 旋转 0°, ±15°, ±30° | 3-5张 |

### 验收标准

| 重投影误差 | 评级 | 说明 |
|-----------|------|------|
| < 0.15 px | 优秀 | 满足 ±0.5mm 焊接定位 |
| < 0.30 px | 良好 | 满足要求 |
| < 0.50 px | 可接受 | 建议优化 |
| > 0.50 px | 差 | 需要重新标定 |

---

## 项目结构

`
3D-Weld-Seam-Tracking-using-PCL/
├── modules/
│   ├── m1_camera_calibration/         # M1: 相机内参标定
│   │   ├── include/
│   │   │   ├── calibration_types.hpp        # 数据类型
│   │   │   ├── camera_interface.hpp         # 相机抽象接口
│   │   │   ├── camera_calibration.hpp       # 标定器核心
│   │   │   └── camera_calibration_export.h  # DLL导出(C#集成)
│   │   ├── src/
│   │   │   ├── camera_calibration.cpp       # 标定实现
│   │   │   ├── camera_calibration_export.cpp# DLL导出实现
│   │   │   └── main.cpp                     # 命令行入口
│   │   ├── config/
│   │   │   └── calibration_config.json      # 默认配置
│   │   └── CMakeLists.txt
│   │
│   └── m2_depth_to_pointcloud/         # M2: 深度图→点云
│       ├── include/
│       │   └── depth_to_pointcloud.hpp       # 点云转换器
│       ├── src/
│       │   ├── depth_to_pointcloud.cpp       # 转换实现
│       │   └── main.cpp                      # 命令行入口
│       └── CMakeLists.txt
│
├── docs/
│   └── images/
├── CMakeLists.txt                     # 根构建配置
├── vcpkg.json                         # 依赖管理
├── .gitignore
└── README.md
`

---

## 集成到 WPF 上位机

### 方案 A: C DLL + C# P/Invoke (推荐)

`csharp
[DllImport("M1_CalibrationLib.dll", CallingConvention = CallingConvention.Cdecl)]
public static extern int M1_CalibrateFromFiles(
    string[] imagePaths, int imageCount, string outputPath,
    int boardWidth, int boardHeight, double squareSizeMm,
    out double fx, out double fy, out double cx, out double cy,
    out double k1, out double k2, out double p1, out double p2, out double k3,
    out double reprojectionError);
`

### 方案 B: C++/CLI 包装器

创建 C++/CLI 项目作为桥接层，直接暴露 .NET 接口给 C# 调用。

### 方案 C: 纯 C# 实现（OpenCVSharp）

`csharp
using OpenCvSharp;
var cameraMatrix = Cv2.InitCameraMatrix2D(objectPoints, imagePoints, imageSize);
`

---

## 开发路线图

- [x] M1: 相机内参标定（含畸变校正）
- [x] M2: 深度图→点云生成
- [ ] M3: 点云滤波（降采样/去噪） — 下一阶段
- [ ] M4: 焊缝特征提取与路径规划
- [ ] M5: 手眼标定
- [ ] M6: C++ DLL 封装 + WPF 集成

---

## License

MIT License

Copyright (c) 2026
