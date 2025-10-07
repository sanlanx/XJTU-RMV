cmake_minimum_required(VERSION 3.8)
project(hik_camera_driver)

# 默认使用C++17标准
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# 设置MVS SDK路径
set(MVS_INCLUDE_DIRS /opt/MVS/include)
set(MVS_LIBRARY_DIRS /opt/MVS/lib/64)

# 添加头文件目录
include_directories(
  include
  ${MVS_INCLUDE_DIRS}
)

# 链接目录
link_directories(${MVS_LIBRARY_DIRS})

# 创建测试程序
add_executable(test_mvs src/test_mvs.cpp)
target_link_libraries(test_mvs MvCameraControl)

# 创建相机驱动节点
add_executable(hik_camera_node 
  src/hik_camera_node.cpp
  src/hik_camera.cpp
)

# 链接ROS2依赖
ament_target_dependencies(hik_camera_node
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
  std_msgs
  std_srvs
)

# 链接MVS库
target_link_libraries(hik_camera_node
  MvCameraControl
)

# 安装规则
install(TARGETS 
  hik_camera_node
  test_mvs
  DESTINATION lib/${PROJECT_NAME}
)

# 安装启动文件和配置文件
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()