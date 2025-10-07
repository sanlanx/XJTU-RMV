#include "hik_camera_driver/hik_camera.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
    // 初始化ROS2
    rclcpp::init(argc, argv);
    
    try {
        // 创建相机节点
        auto camera_node = std::make_shared<HikCamera>();
        
        // 初始化相机硬件
        if (camera_node->initialize()) {
            // 采集图像
            camera_node->startStreaming();
            RCLCPP_INFO(camera_node->get_logger(), "HikCamera node started successfully");
            
            // 事件循环，保持节点运行
            rclcpp::spin(camera_node);
            
            // 程序退出时停止采集
            camera_node->stopStreaming();
        } else {
            RCLCPP_ERROR(camera_node->get_logger(), "Failed to initialize camera");
        }
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("hik_camera"), "Exception in main: %s", e.what());
        return 1;
    }
    
    // 关闭ROS2
    rclcpp::shutdown();
    return 0;
}