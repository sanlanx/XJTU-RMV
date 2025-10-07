## 程序功能与实现
### 运行程序
```
colcon build
source install/setup.bash
ros2 launch hik_camera_driver camera.launch.py
```
这时会开启相机，同时启动rviz2   

### 功能实现
1.支持相机通过USB 3.0接口连接，支持相机断线重连

2.能够稳定的采集图像数据，并通过rviz2可视化，在rviz2界面点击Add -> By toppic -> image -> OK就可以看到相机获取的图片

3.支持参数读取与配置：  
通过`ros2 param set /hik_camera <属性> <数值>`的方式进行参数配置,参数包括：  
图像尺寸（width,height）、增益（gain）、采集帧率（frame_rate）、图像格式（pixel_format）、曝光（exposure_time）  
通过`ros2 param get /hik_camera <属性>`的方式进行参数读取，实际采集帧率可以从运行的终端实时获取

4.最大帧率实现
在曝光为6000时，帧率可达到161.97FPS（由于相机少的原因没能测165FPS时曝光时多少）  

## 检测相机信息与SDK获取
### 相机信息以及接口设置界面
```
cd /opt/MVS/bin
sudo ./MVS.sh
```

### SDK官网
file:///home/sanlan/Downloads/MVS_Linux_STD_V4.6.0_250808/index.html
通过这个文档能够查询参数以及函数配置

## 问题与解决
### 安装ROS2
遇到系统缺少ROS2仓库的GPG公钥问题，导致无法验证软件包的安全性  
运行`sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys F42ED6FBAB17C654`添加GPG公钥

### 获取MVS C++ SDK
访问连接：https://www.hikrobotics.com/cn/machinevision/service/download/?module=0  
下载相应的文件并安装

### 最高帧率限制问题
在首次完成程序后，发现最大帧率受到了限制，限制在83.43  
请教了高组长，原因是图片格式问题，将图片格式从“bgr8”改为“”bayerrg8"后帧率得到提高  
同时还对源代码进行了修改，将采集图片与通信分离，实现通信耗时的减少