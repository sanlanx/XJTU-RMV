#pragma once  // 防止重复包含

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <cv_bridge/cv_bridge.h>
#include <atomic>
#include <thread>

#include "MvCameraControl.h"

class HikCamera : public rclcpp::Node
{
public:
    HikCamera();
    ~HikCamera();

    bool initialize();  // 初始化相机
    void startStreaming();  // 开始采集
    void stopStreaming();  // 停止采集

    double getCurrentFrameRate();
    double getAcquisitionFrameRate(); // 获取设定的帧率

private:
    bool connectToCamera();  // 连接相机
    void disconnectCamera();  // 发布图像
    void setupCamera();  // 配置相机参数
    void publishImage(const unsigned char* pData, unsigned int nDataSize, 
                     unsigned int nWidth, unsigned int nHeight);
    
    // 采集线程函数
    void captureThread();
    std::thread capture_thread_;
    std::atomic<bool> capture_running_;
    
    // 参数回调函数
    rcl_interfaces::msg::SetParametersResult parametersCallback(
        const std::vector<rclcpp::Parameter> &parameters);
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
    
    // 重连机制
    void startReconnectionTimer();
    void reconnectionTimerCallback();

    void handleConnectionLost();
    bool was_streaming_before_disconnect_;
    
    // ROS2组件
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr reconnection_timer_;
    image_transport::Publisher image_pub_;

    // 海康相机组件
    void* camera_handle_;  // 相机句柄
    bool is_connected_;
    bool is_streaming_;  // 采集状态
    
    // 参数
    int device_index_;
    std::string camera_serial_;  // 相机序列号
    std::string camera_ip_;      // 相机IP地址
    double frame_rate_;
    int width_;
    int height_;
    double exposure_time_;
    double gain_;
    std::string pixel_format_;
    bool auto_reconnect_;
    double reconnect_interval_;
    
    // 设备信息
    MV_CC_DEVICE_INFO current_device_info_;

    // 错误计数
    int error_count_ = 0;
};