from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 获取参数文件路径
    config_file = os.path.join(
        get_package_share_directory('hik_camera_driver'),
        'config',
        'camera_params.yaml'
    )
    
    # 创建相机节点
    camera_node = Node(
        package='hik_camera_driver',
        executable='hik_camera_node',
        name='hik_camera',
        output='screen',
        parameters=[config_file]
    )
    
    # 创建 RViz2 节点（使用默认配置）
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', ''],  # 空字符串表示使用默认配置
        output='screen'
    )
    
    # 启动 RViz2，确保相机节点已启动
    delayed_rviz2 = TimerAction(
        period=0.0,
        actions=[rviz2_node]
    )
    
    return LaunchDescription([
        camera_node,
        delayed_rviz2
    ])