#include "hik_camera_driver/hik_camera.hpp"
#include <iostream>
#include <chrono>
#include <memory>
#include <thread>

HikCamera::HikCamera() 
    : Node("hik_camera"), camera_handle_(nullptr), is_connected_(false), is_streaming_(false),
      device_index_(0), frame_rate_(30.0), width_(1280), height_(1024),
      exposure_time_(1000.0), gain_(0.0), pixel_format_("bayerrg8"),
      auto_reconnect_(true), reconnect_interval_(5.0),
      was_streaming_before_disconnect_(false)
{
    // 声明参数
    this->declare_parameter<int>("device_index", 0);
    this->declare_parameter<std::string>("camera_serial", "");
    this->declare_parameter<std::string>("camera_ip", "");
    this->declare_parameter<double>("frame_rate", 30.0);
    this->declare_parameter<int>("width", 1280);
    this->declare_parameter<int>("height", 1024);
    this->declare_parameter<double>("exposure_time", 1000.0);
    this->declare_parameter<double>("gain", 0.0);
    this->declare_parameter<std::string>("pixel_format", "bayerrg8");
    this->declare_parameter<bool>("auto_reconnect", true);
    this->declare_parameter<double>("reconnect_interval", 5.0);
    
    // 获取参数
    device_index_ = this->get_parameter("device_index").as_int();
    camera_serial_ = this->get_parameter("camera_serial").as_string();
    camera_ip_ = this->get_parameter("camera_ip").as_string();
    frame_rate_ = this->get_parameter("frame_rate").as_double();
    width_ = this->get_parameter("width").as_int();
    height_ = this->get_parameter("height").as_int();
    exposure_time_ = this->get_parameter("exposure_time").as_double();
    gain_ = this->get_parameter("gain").as_double();
    pixel_format_ = this->get_parameter("pixel_format").as_string();
    auto_reconnect_ = this->get_parameter("auto_reconnect").as_bool();
    reconnect_interval_ = this->get_parameter("reconnect_interval").as_double();
    
    // 创建图像发布者
    image_pub_ = image_transport::create_publisher(this, "image_raw");
    
    // 设置参数回调
    param_callback_handle_ = this->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> &parameters) {
            return this->parametersCallback(parameters);
        });
    
    // RCLCPP_INFO(this->get_logger(), "HikCamera node initialized");
    RCLCPP_INFO(this->get_logger(), "Device: %d, Frame rate: %.1f, Resolution: %dx%d", 
                device_index_, frame_rate_, width_, height_);
}

HikCamera::~HikCamera()
{
    stopStreaming();
    disconnectCamera();
}

bool HikCamera::initialize()
{
    return connectToCamera();
}

bool HikCamera::connectToCamera()
{
    RCLCPP_INFO(this->get_logger(), "Connecting to USB camera...");
    
    // 枚举USB设备
    MV_CC_DEVICE_INFO_LIST stDeviceList;
    memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
    
    int nRet = MV_CC_EnumDevices(MV_USB_DEVICE, &stDeviceList);
    if (MV_OK != nRet) {
        RCLCPP_ERROR(this->get_logger(), "Enum USB devices fail! nRet: 0x%x", nRet);
        if (auto_reconnect_) {
            startReconnectionTimer();
        }
        return false;
    }
    
    if (stDeviceList.nDeviceNum == 0) {
        RCLCPP_ERROR(this->get_logger(), "No USB camera found!");
        if (auto_reconnect_) {
            startReconnectionTimer();
        }
        return false;
    }
    
    RCLCPP_INFO(this->get_logger(), "Found %d USB cameras", stDeviceList.nDeviceNum);
    
    // 根据序列号查找相机
    int target_index = device_index_;
    if (!camera_serial_.empty() || !camera_ip_.empty()) {
        for (unsigned int i = 0; i < stDeviceList.nDeviceNum; i++) {
            MV_CC_DEVICE_INFO* pDeviceInfo = stDeviceList.pDeviceInfo[i];
            
            if (pDeviceInfo->nTLayerType == MV_USB_DEVICE) {
                // 检查序列号
                if (!camera_serial_.empty()) {
                    std::string serial(reinterpret_cast<char*>(pDeviceInfo->SpecialInfo.stUsb3VInfo.chSerialNumber));
                    if (serial == camera_serial_) {
                        target_index = i;
                        RCLCPP_INFO(this->get_logger(), "Found camera by serial: %s", camera_serial_.c_str());
                        break;
                    }
                }
            }
        }
    }
    
    // 检查设备索引是否有效
    if (target_index >= static_cast<int>(stDeviceList.nDeviceNum)) {
        RCLCPP_ERROR(this->get_logger(), "Device index %d is invalid! Only %d cameras found.", 
                    target_index, stDeviceList.nDeviceNum);
        if (auto_reconnect_) {
            startReconnectionTimer();
        }
        return false;
    }
    
    // 连接指定索引的USB相机
    nRet = MV_CC_CreateHandle(&camera_handle_, stDeviceList.pDeviceInfo[target_index]);
    if (MV_OK != nRet) {
        RCLCPP_ERROR(this->get_logger(), "Create handle fail! nRet: 0x%x", nRet);
        if (auto_reconnect_) {
            startReconnectionTimer();
        }
        return false;
    }
    
    nRet = MV_CC_OpenDevice(camera_handle_);
    if (MV_OK != nRet) {
        RCLCPP_ERROR(this->get_logger(), "Open USB device fail! nRet: 0x%x", nRet);
        MV_CC_DestroyHandle(camera_handle_);
        camera_handle_ = nullptr;
        if (auto_reconnect_) {
            startReconnectionTimer();
        }
        return false;
    }
    
    // 保存设备信息
    memcpy(&current_device_info_, stDeviceList.pDeviceInfo[target_index], sizeof(MV_CC_DEVICE_INFO));
    
    // 设置相机参数
    setupCamera();
    
    is_connected_ = true;
    RCLCPP_INFO(this->get_logger(), "USB camera connected successfully");
    
    // 停止重连定时器
    if (reconnection_timer_) {
        reconnection_timer_->cancel();
    }
    
    return true;
}

void HikCamera::setupCamera()
{
    if (!camera_handle_) return;
    
    int nRet = MV_OK;
    
    // 设置触发模式为连续采集
    nRet = MV_CC_SetEnumValue(camera_handle_, "TriggerMode", 0);
    
    // 设置图像尺寸
    nRet = MV_CC_SetIntValue(camera_handle_, "Width", width_);
    
    nRet = MV_CC_SetIntValue(camera_handle_, "Height", height_);
    
    // 设置像素格式
    unsigned int pixel_format = PixelType_Gvsp_BGR8_Packed;
    if (pixel_format_ == "mono8") {
        pixel_format = PixelType_Gvsp_Mono8;
    } else if (pixel_format_ == "mono16") {
        pixel_format = PixelType_Gvsp_Mono16;
    } else if (pixel_format_ == "bgr8") {
        pixel_format = PixelType_Gvsp_BGR8_Packed;
    } else if (pixel_format_ == "rgb8") {
        pixel_format = PixelType_Gvsp_RGB8_Packed;
    } else if (pixel_format_ == "bayerrg8") {
        pixel_format = PixelType_Gvsp_BayerRG8;
    }
    
    nRet = MV_CC_SetEnumValue(camera_handle_, "PixelFormat", pixel_format);
    
    // 设置曝光时间
    nRet = MV_CC_SetFloatValue(camera_handle_, "ExposureTime", exposure_time_);
    
    // 设置增益
    nRet = MV_CC_SetFloatValue(camera_handle_, "Gain", gain_);

    // 设置采集帧率
    nRet = MV_CC_SetFloatValue(camera_handle_, "AcquisitionFrameRate", frame_rate_);
    if (MV_OK != nRet) {
        RCLCPP_WARN(this->get_logger(), "Set AcquisitionFrameRate failed! nRet: 0x%x", nRet);
    } else {
        // 启用帧率控制
        nRet = MV_CC_SetBoolValue(camera_handle_, "AcquisitionFrameRateEnable", true);
    }
}

rcl_interfaces::msg::SetParametersResult HikCamera::parametersCallback(
    const std::vector<rclcpp::Parameter> &parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    
    for (const auto &parameter : parameters) {
        if (parameter.get_name() == "exposure_time") {
            exposure_time_ = parameter.as_double();
            if (is_connected_ && camera_handle_) {
                int nRet = MV_CC_SetFloatValue(camera_handle_, "ExposureTime", exposure_time_);
                if (MV_OK == nRet) {
                    RCLCPP_INFO(this->get_logger(), "Exposure time set to: %.1f", exposure_time_);
                } else {
                    RCLCPP_WARN(this->get_logger(), "Set ExposureTime failed! nRet: 0x%x", nRet);
                    result.successful = false;
                    result.reason = "Failed to set exposure time";
                }
            }
        } else if (parameter.get_name() == "gain") {
            gain_ = parameter.as_double();
            if (is_connected_ && camera_handle_) {
                int nRet = MV_CC_SetFloatValue(camera_handle_, "Gain", gain_);
                if (MV_OK == nRet) {
                    RCLCPP_INFO(this->get_logger(), "Gain set to: %.1f", gain_);
                } else {
                    RCLCPP_WARN(this->get_logger(), "Set Gain failed! nRet: 0x%x", nRet);
                    result.successful = false;
                    result.reason = "Failed to set gain";
                }
            }
        } else if (parameter.get_name() == "frame_rate") {
            double new_frame_rate = parameter.as_double();
            if (new_frame_rate <= 0) {
                result.successful = false;
                result.reason = "Frame rate must be positive";
                return result;
            }
            
            if (is_connected_ && camera_handle_) {
                // 首先设置采集帧率
                int nRet = MV_CC_SetFloatValue(camera_handle_, "AcquisitionFrameRate", new_frame_rate);
                if (MV_OK == nRet) {
                    frame_rate_ = new_frame_rate;
                    
                    // 启用帧率控制
                    nRet = MV_CC_SetBoolValue(camera_handle_, "AcquisitionFrameRateEnable", true);
                    if (MV_OK != nRet) {
                        RCLCPP_WARN(this->get_logger(), "Enable AcquisitionFrameRateEnable failed! nRet: 0x%x", nRet);
                    }
                    
                    // 获取并显示实际帧率
                    // std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 等待设置生效
                    double actual_frame_rate = getCurrentFrameRate();
                    // double target_frame_rate = getAcquisitionFrameRate();
                    
                    RCLCPP_INFO(this->get_logger(), 
                            "Frame rate set - Target: %.1f FPS, Actual: %.2f FPS", 
                            new_frame_rate, actual_frame_rate);
                            
                } else {
                    RCLCPP_WARN(this->get_logger(), "Set AcquisitionFrameRate failed! nRet: 0x%x", nRet);
                    result.successful = false;
                    result.reason = "Failed to set frame rate";
                }
            }
        } else if (parameter.get_name() == "pixel_format") {
            pixel_format_ = parameter.as_string();
            RCLCPP_INFO(this->get_logger(), "Pixel format changed to: %s", pixel_format_.c_str());
            // 需要重启流来应用新的像素格式
            if (is_streaming_) {
                stopStreaming();
                startStreaming();
            }
        }
    }
    
    return result;
}

void HikCamera::startReconnectionTimer()
{
    if (!auto_reconnect_) return;
    
    reconnection_timer_ = this->create_wall_timer(
        std::chrono::seconds(static_cast<int>(reconnect_interval_)),
        std::bind(&HikCamera::reconnectionTimerCallback, this)
    );
    RCLCPP_INFO(this->get_logger(), "Reconnection timer started, will retry in %.1f seconds", reconnect_interval_);
}

void HikCamera::reconnectionTimerCallback()
{
    RCLCPP_INFO(this->get_logger(), "Attempting to reconnect to camera...");
    
    // 确保之前的连接完全清理
    if (camera_handle_) {
        try {
            MV_CC_DestroyHandle(camera_handle_);
        } catch (...) {
            // 忽略异常
        }
        camera_handle_ = nullptr;
    }
    
    if (connectToCamera()) {
        RCLCPP_INFO(this->get_logger(), "Reconnection successful!");
        // 重连后自动恢复流传输
        if (was_streaming_before_disconnect_) {
            startStreaming();
        }
    } else {
        RCLCPP_WARN(this->get_logger(), "Reconnection failed, will retry in %.1f seconds", reconnect_interval_);
    }
}

void HikCamera::disconnectCamera()
{
    if (camera_handle_) {
        if (is_streaming_) {
            stopStreaming();
        }
        
        MV_CC_CloseDevice(camera_handle_);
        MV_CC_DestroyHandle(camera_handle_);
        camera_handle_ = nullptr;
        is_connected_ = false;
        RCLCPP_INFO(this->get_logger(), "USB camera disconnected");
    }
}
/*
void HikCamera::startStreaming()
{
    if (!is_connected_ || !camera_handle_) {
        RCLCPP_ERROR(this->get_logger(), "USB camera not connected!");
        return;
    }
    
    // RCLCPP_INFO(this->get_logger(), "Setting up streaming parameters...");

    was_streaming_before_disconnect_ = true;  // 记录流状态

    // 设置采集模式为连续
    int nRet = MV_CC_SetEnumValue(camera_handle_, "AcquisitionMode", 2);
    
    // 确保触发模式设置为关闭（连续采集）
    nRet = MV_CC_SetEnumValue(camera_handle_, "TriggerMode", 0);

    // 开始取流
    nRet = MV_CC_StartGrabbing(camera_handle_);
    if (MV_OK != nRet) return;
    
    RCLCPP_INFO(this->get_logger(), "USB camera streaming started");
    is_streaming_ = true;
    
    // 使用定时器轮询方式获取图像
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1), // ~30fps
        [this]() {
            if (!is_streaming_ || !camera_handle_) return;
            
            MV_FRAME_OUT stImageInfo = {0};
            memset(&stImageInfo, 0, sizeof(MV_FRAME_OUT));
            
            int nRet = MV_CC_GetImageBuffer(camera_handle_, &stImageInfo, 100);
            
            if (nRet == MV_OK) {
                RCLCPP_DEBUG(this->get_logger(), "Got image: %dx%d, size: %d bytes", 
                           stImageInfo.stFrameInfo.nWidth, stImageInfo.stFrameInfo.nHeight, 
                           stImageInfo.stFrameInfo.nFrameLen);
                publishImage(stImageInfo.pBufAddr, stImageInfo.stFrameInfo.nFrameLen,
                           stImageInfo.stFrameInfo.nWidth, stImageInfo.stFrameInfo.nHeight);
                MV_CC_FreeImageBuffer(camera_handle_, &stImageInfo);
            } else {
                RCLCPP_DEBUG(this->get_logger(), "Get image buffer returned: 0x%x", nRet);
                
                // 通用方法：连续多次失败则认为连接断开
                static int error_count = 0;
                if (nRet != MV_OK) {
                    error_count++;
                    if (error_count >= 5) {  // 连续5次失败
                        RCLCPP_ERROR(this->get_logger(), "Camera connection lost after %d consecutive errors! Last error: 0x%x", 
                                   error_count, nRet);
                        error_count = 0;  // 重置计数器
                        handleConnectionLost();
                    }
                } else {
                    error_count = 0;  // 成功时重置计数器
                }
            }
        }
    );
}

void HikCamera::stopStreaming()
{
    if (is_streaming_ && camera_handle_) {
        was_streaming_before_disconnect_ = false;  // 手动停止时不记录状态
        MV_CC_StopGrabbing(camera_handle_);
        if (timer_) {
            timer_->cancel();
            timer_.reset();
        }
        is_streaming_ = false;
    }
}*/

void HikCamera::startStreaming() {
    if (!is_connected_ || !camera_handle_) {
        RCLCPP_ERROR(this->get_logger(), "USB camera not connected!");
        return;
    }

    int nRet = MV_CC_StartGrabbing(camera_handle_);
    if (MV_OK != nRet) return;
    
    is_streaming_ = true;
    capture_running_ = true;
    
    // 启动采集线程
    capture_thread_ = std::thread(&HikCamera::captureThread, this);
    
    RCLCPP_INFO(this->get_logger(), "USB camera streaming started (dedicated thread)");
}

void HikCamera::stopStreaming() {
    if (is_streaming_ && camera_handle_) {
        capture_running_ = false;
        is_streaming_ = false;
        
        if (capture_thread_.joinable()) {
            capture_thread_.join();
        }
        
        MV_CC_StopGrabbing(camera_handle_);
        RCLCPP_INFO(this->get_logger(), "USB camera streaming stopped");
    }
}

void HikCamera::publishImage(const unsigned char* pData, unsigned int nDataSize, 
                           unsigned int nWidth, unsigned int nHeight)
{
    try {
        auto image_msg = std::make_shared<sensor_msgs::msg::Image>();
        image_msg->header.stamp = this->now();
        image_msg->header.frame_id = "hik_camera";
        image_msg->height = nHeight;
        image_msg->width = nWidth;
        image_msg->encoding = pixel_format_;
        
        // 根据像素格式计算正确的步长
        if (pixel_format_ == "bgr8" || pixel_format_ == "rgb8") {
            image_msg->step = nWidth * 3;  // 3 bytes per pixel
        } else if (pixel_format_ == "mono8") {
            image_msg->step = nWidth;      // 1 byte per pixel
        } else if (pixel_format_ == "mono16") {
            image_msg->step = nWidth * 2;  // 2 bytes per pixel
        } else {
            image_msg->step = nWidth * 3;  // 默认值
        }
        
        image_msg->data.resize(nDataSize);
        memcpy(image_msg->data.data(), pData, nDataSize);
        
        image_pub_.publish(image_msg);
        
        // 偶尔打印详细的相机状态信息（避免过于频繁）
        static int count = 0;
        if (count++ % 30 == 0) {
            // 获取当前实际帧率
            double actual_frame_rate = getCurrentFrameRate();
            
            // 获取当前曝光时间和增益
            double current_exposure = exposure_time_;
            double current_gain = gain_;
            
            // 如果相机连接正常，尝试获取实际参数值
            if (is_connected_ && camera_handle_) {
                MVCC_FLOATVALUE stFloatValue = {0};
                
                // 获取实际曝光时间
                int nRet = MV_CC_GetFloatValue(camera_handle_, "ExposureTime", &stFloatValue);
                if (MV_OK == nRet) {
                    current_exposure = stFloatValue.fCurValue;
                }
                
                // 获取实际增益
                nRet = MV_CC_GetFloatValue(camera_handle_, "Gain", &stFloatValue);
                if (MV_OK == nRet) {
                    current_gain = stFloatValue.fCurValue;
                }
            }
            
            RCLCPP_INFO(this->get_logger(), 
                "Camera Status - Resolution: %dx%d, Frame Rate: %.2f FPS, Format: %s, "
                "Exposure: %.1f μs, Gain: %.1f dB", 
                nWidth, nHeight, actual_frame_rate, pixel_format_.c_str(),
                current_exposure, current_gain);
        }
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error publishing image: %s", e.what());
    }
}

void HikCamera::handleConnectionLost()
{
    RCLCPP_ERROR(this->get_logger(), "Handling camera connection loss...");
    
    // 标记连接断开
    is_connected_ = false;
    is_streaming_ = false;
    
    // 停止定时器
    if (timer_) {
        timer_->cancel();
        timer_.reset();
    }
    
    // 清理相机句柄
    if (camera_handle_) {
        // 尝试优雅关闭
        try {
            MV_CC_StopGrabbing(camera_handle_);
            MV_CC_CloseDevice(camera_handle_);
            MV_CC_DestroyHandle(camera_handle_);
        } catch (...) {
            // 忽略清理时的异常
        }
        camera_handle_ = nullptr;
    }
    
    // 启动重连
    if (auto_reconnect_) {
        startReconnectionTimer();
    } else {
        RCLCPP_ERROR(this->get_logger(), "Camera connection lost and auto_reconnect is disabled");
    }
}

double HikCamera::getCurrentFrameRate()
{
    if (!camera_handle_ || !is_connected_) {
        RCLCPP_WARN(this->get_logger(), "Camera not connected, cannot get frame rate");
        return 0.0;
    }

    int nRet = MV_OK;
    MVCC_FLOATVALUE stFloatValue = {0};
    
    // 尝试多种方式获取帧率
    const char* frame_rate_params[] = {
        "ResultingFrameRate",      // 实时输出帧率
        // "AcquisitionFrameRate",    // 采集帧率设定值
        // "DeviceFrameRate"          // 设备帧率
    };
    
    for (const char* param : frame_rate_params) {
        nRet = MV_CC_GetFloatValue(camera_handle_, param, &stFloatValue);
        if (MV_OK == nRet && stFloatValue.fCurValue > 0) {
            return static_cast<double>(stFloatValue.fCurValue);
        }
    }
    return 0.0;
}

double HikCamera::getAcquisitionFrameRate()
{
    if (!camera_handle_ || !is_connected_) {
        return 0.0;
    }

    int nRet = MV_OK;
    MVCC_FLOATVALUE stFloatValue = {0};

    nRet = MV_CC_GetFloatValue(camera_handle_, "AcquisitionFrameRate", &stFloatValue);
    if (MV_OK == nRet) {
        return static_cast<double>(stFloatValue.fCurValue);
    }
    return 0.0;
}

void HikCamera::captureThread() {
    RCLCPP_INFO(this->get_logger(), "Capture thread started");
    
    while (capture_running_ && rclcpp::ok()) {
        if (!is_streaming_ || !camera_handle_) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }
        
        MV_FRAME_OUT stImageInfo = {0};
        memset(&stImageInfo, 0, sizeof(MV_FRAME_OUT));
        
        int nRet = MV_CC_GetImageBuffer(camera_handle_, &stImageInfo, 10);
        
        if (nRet == MV_OK) {
            publishImage(stImageInfo.pBufAddr, stImageInfo.stFrameInfo.nFrameLen,
                       stImageInfo.stFrameInfo.nWidth, stImageInfo.stFrameInfo.nHeight);
            MV_CC_FreeImageBuffer(camera_handle_, &stImageInfo);
            error_count_ = 0;
        } else {
            error_count_++;
            if (error_count_ >= 10) {
                RCLCPP_ERROR(this->get_logger(), "Camera connection lost");
                error_count_ = 0;
                handleConnectionLost();
            }
        }
        
        // 根据帧率控制采集频率
        int interval = static_cast<int>(1000.0 / frame_rate_);
        if (interval > 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(interval));
        }
    }
    RCLCPP_INFO(this->get_logger(), "Capture thread stopped");
}