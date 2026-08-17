#ifndef CAMERA_INTERFACE_HPP
#define CAMERA_INTERFACE_HPP

/**
 * @file camera_interface.hpp
 * @brief M1 相机标定模块 - 相机抽象接口
 * @description 定义相机 SDK 的抽象接口，支持多种相机接入。
 *              实际使用时，需要根据具体相机 SDK 实现此接口。
 *
 * 选型建议（给 C# 上位机开发者）：
 * 1. Hikvision MVS （海康）：工业相机市场占有率高，SDK 完善，推荐
 * 2. Basler Pylon  ：德国品牌，SDK 稳定，文档好
 * 3. Daheng Galaxy  ：大恒相机，性价比高
 * 4. 自定义 USB 相机：用 OpenCV VideoCapture 即可
 *
 * 集成方式：
 * - 方案 A（推荐）：C++ 实现此接口，编译为 DLL，C# 通过 P/Invoke 调用
 * - 方案 B：C# 端通过 OpenCVSharp 直接调用 OpenCV 标定函数
 * - 方案 C：C++ CLI 混合程序集，直接桥接 C# 和 C++
 *
 * 本模块采用方案 A，提供 C 风格导出函数（见 camera_calibration.hpp）。
 */

#include <opencv2/opencv.hpp>
#include <string>

namespace bus_welding {

/// 相机像素格式
enum class PixelFormat {
    MONO8,        ///< 8位灰度
    MONO12,       ///< 12位灰度
    BAYER_RG8,    ///< Bayer RG 8位
    BAYER_RG12,   ///< Bayer RG 12位
    RGB8,         ///< 24位彩色
    BGR8,         ///< 24位彩色 (BGR顺序)
    YUV422,       ///< YUV 4:2:2
    UNKNOWN       ///< 未知格式
};

/// 触发模式
enum class TriggerMode {
    CONTINUOUS,   ///< 连续采集
    SOFT_TRIGGER, ///< 软触发
    HARD_TRIGGER, ///< 硬触发 (外部IO)
};

/**
 * @brief 相机抽象接口
 *
 * 所有具体的相机 SDK 实现都需要继承此类。
 * 标定模块通过此接口采集图像，无需关心底层相机型号。
 *
 * 实现示例（Hikvision）：
 * class HikCamera : public CameraInterface {
 *     // 实现 open/close/grabFrame/setParameter 等
 * };
 */
class CameraInterface {
public:
    virtual ~CameraInterface() = default;

    /// 打开相机
    virtual bool open(const std::string& camera_id) = 0;

    /// 关闭相机
    virtual void close() = 0;

    /// 判断相机是否已打开
    virtual bool isOpen() const = 0;

    /// 采集一帧图像
    /// @param frame 输出图像 (BGR 格式)
    /// @return true 成功, false 失败
    virtual bool grabFrame(cv::Mat& frame) = 0;

    /// 获取相机名称（用于日志和显示）
    virtual std::string getCameraName() const = 0;

    /// 获取相机序列号
    virtual std::string getSerialNumber() const = 0;

    /// 设置曝光时间 (us)
    virtual bool setExposureTime(double exposure_us) = 0;

    /// 获取曝光时间 (us)
    virtual double getExposureTime() const = 0;

    /// 设置增益 (dB)
    virtual bool setGain(double gain_db) = 0;

    /// 获取增益 (dB)
    virtual double getGain() const = 0;

    /// 设置触发模式
    virtual bool setTriggerMode(TriggerMode mode) = 0;

    /// 获取相机信息
    virtual CameraInfo getCameraInfo() const = 0;

    /// 获取当前图像尺寸
    virtual cv::Size getImageSize() const = 0;

    /// 设置感兴趣区域 (ROI)
    virtual bool setROI(int x, int y, int width, int height) = 0;
};

/**
 * @brief OpenCV VideoCapture 实现（用于测试和 USB 相机）
 *
 * 如果使用 USB 相机或笔记本摄像头，可以这样用：
 *   OpenCVCamera cam(0);  // 0 = 默认摄像头
 *   cv::Mat frame;
 *   cam.grabFrame(frame);
 */
class OpenCVCamera : public CameraInterface {
public:
    /// @param camera_index 相机索引 (0, 1, 2, ...) 或 RTSP URL
    explicit OpenCVCamera(int camera_index = 0)
        : camera_index_(camera_index) {}

    explicit OpenCVCamera(const std::string& rtsp_url)
        : camera_index_(0), rtsp_url_(rtsp_url), use_rtsp_(true) {}

    ~OpenCVCamera() override { close(); }

    bool open(const std::string& /*camera_id*/) override {
        if (use_rtsp_) {
            cap_.open(rtsp_url_);
        } else {
            cap_.open(camera_index_);
        }
        if (!cap_.isOpened()) {
            return false;
        }
        // 设置较大的缓冲区，避免丢帧
        cap_.set(cv::CAP_PROP_BUFFERSIZE, 3);
        return true;
    }

    void close() override {
        if (cap_.isOpened()) {
            cap_.release();
        }
    }

    bool isOpen() const override {
        return cap_.isOpened();
    }

    bool grabFrame(cv::Mat& frame) override {
        if (!cap_.isOpened()) return false;
        return cap_.read(frame);
    }

    std::string getCameraName() const override {
        if (use_rtsp_) return "OpenCV_RTSP:" + rtsp_url_;
        return "OpenCV_Camera:" + std::to_string(camera_index_);
    }

    std::string getSerialNumber() const override {
        return "OpenCV_" + std::to_string(camera_index_);
    }

    bool setExposureTime(double exposure_us) override {
        // OpenCV 曝光单位是毫秒
        return cap_.set(cv::CAP_PROP_EXPOSURE, exposure_us / 1000.0);
    }

    double getExposureTime() const override {
        return cap_.get(cv::CAP_PROP_EXPOSURE) * 1000.0;
    }

    bool setGain(double gain_db) override {
        return cap_.set(cv::CAP_PROP_GAIN, gain_db);
    }

    double getGain() const override {
        return cap_.get(cv::CAP_PROP_GAIN);
    }

    bool setTriggerMode(TriggerMode mode) override {
        // OpenCV VideoCapture 不支持触发模式
        (void)mode;
        return false;
    }

    CameraInfo getCameraInfo() const override {
        CameraInfo info;
        info.vendor = "OpenCV";
        info.model = "VideoCapture";
        info.pixel_width = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_WIDTH));
        info.pixel_height = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_HEIGHT));
        return info;
    }

    cv::Size getImageSize() const override {
        int w = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_WIDTH));
        int h = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_HEIGHT));
        return cv::Size(w, h);
    }

    bool setROI(int x, int y, int width, int height) override {
        // OpenCV 不支持直接设置 ROI
        (void)x; (void)y; (void)width; (void)height;
        return false;
    }

private:
    cv::VideoCapture cap_;
    int camera_index_ = 0;
    std::string rtsp_url_;
    bool use_rtsp_ = false;
};

} // namespace bus_welding

#endif // CAMERA_INTERFACE_HPP