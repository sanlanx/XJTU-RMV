# 问题与解决
## 下载ceres
刚开始遇到来自`tbb_stddef.h`和`gtest`的报错  
参照：https://axi404.top/blog/ceres-install 得到解决

## 编译问题
在CMakeLists.txt文件中添加依赖并连接  
```
# 查找必需的包
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)

# 使用传统方式链接库
target_include_directories(task PRIVATE
    ${EIGEN3_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
)

target_link_libraries(task
    ${CERES_LIBRARIES}
    ${EIGEN3_LIBRARIES}
)
```

## 视频无法打开
路径正确但是视频却无法打开，原因是缺少FFmpeg支持  
解决方案(重装OpenCV):  
```
# 卸载已安装的 OpenCV
sudo make uninstall

# 安装必要的依赖
sudo apt update
sudo apt install libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
sudo apt install libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libdc1394-22-dev
sudo apt install libv4l-dev v4l-utils qv4l2 v4l2ucp
sudo apt install ffmpeg

# 返回 build 目录
cd opencv-4.8.0/build

# 清理之前的编译结果
rm -rf *

# 重新配置 CMake（确保 FFmpeg 被检测到）
cmake -D WITH_FFMPEG=ON -D WITH_GSTREAMER=ON ..


# 重新编译安装
make -j$(nproc)
sudo make install
```

## 参数拟合问题
下面是我第一次成功拟合的结果：  
```
x0 = 163.799 px
y0 = 128.369 px
vx0 = 252.796 px/s
vy0 = -349.226 px/s
g = -499.453 px/s²
k = 0.066018 1/s

平均误差统计：
X方向平均绝对误差: 0.653252 px
Y方向平均绝对误差: 0.665069 px
X方向平均相对误差: 0.176351%
Y方向平均相对误差: 1.56802%
```
可以看到g的值为负数，是不合理的，了解原因，默认坐标建立为左上角，y轴向下为正  
将坐标原点改到左下角就可以解决

## 轮廓勾勒导致的BUG
在我完成了参数求值后，我对视频进行逐帧勾勒，绘制小球的轮廓，但是此后出现了参数拟合错误问题

我的错误代码：  
```
Point2d ball_center, draw_center;
if (detect_ball(frame, ball_center, draw_center)) {
    double time = frame_count / fps;
    time_points.push_back(time);
    x_positions.push_back(ball_center.x);
    y_positions.push_back(ball_center.y);

    x_positions.push_back(draw_center.x);
    y_positions.push_back(draw_center.y);

        // 可选：在图像上绘制检测到的小球
    circle(frame, draw_center, 5, Scalar(0, 255, 0), 2);
    putText(frame, "Ball", Point(draw_center.x + 10, draw_center.y), 
            FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 1);
}
```

正确的代码：  
```
Point2d ball_center;
Point2d draw_center;
if (detect_ball(frame, ball_center, draw_center)) {
    double time = frame_count / fps;
    time_points.push_back(time);
    x_positions.push_back(ball_center.x);
    y_positions.push_back(ball_center.y);

    // 可选：在图像上绘制检测到的小球
    circle(frame, draw_center, 5, Scalar(0, 255, 0), 2);
    putText(frame, "Ball", Point(draw_center.x + 10, draw_center.y), 
            FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 1);
}
```

原因如下：  
错误代码错把我需要勾勒的轮廓的坐标添加到了positions中，导致参数拟合出错  
现已纠正

# 完成思路
`1、构造参数结构体`  
`2、构造弹道模型函数和残差函数`  
`3、使用OpenCV检测小球（二值化处理）并计算小球球心坐标`  
`4、打开视频并读取每一帧，然后记录小球位置`  
`5、进行曲线拟合并计算平均绝对误差和平均相对误差`

### 结果展示
```
拟合结果：
x0 = 163.799 px
y0 = 591.631 px
vx0 = 252.796 px/s
vy0 = 349.226 px/s
g = 499.453 px/s²
k = 0.0660178 1/s

平均误差统计：
X方向平均绝对误差: 0.653253 px
Y方向平均绝对误差: 0.665069 px
X方向平均相对误差: 0.176352%
Y方向平均相对误差: 0.209672%
```
可以看到拟合效果非常好，平均相对误差远小于3%

# 知识点记录
## 视频下载与播放
视频播放插件：MPV  
`sudo apt install mpv` 下载MPV 
使用`mpv video.mp4`播放

## 初学ceres
### 一维优化
```
struct CostFunctor {
    template <typename T>
    bool operator()(const T* const x, T* residual) const {
        residual[0] = T(10.0) - x[0];
        return true;
    }
};

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    double x = 0.0;
    ceres::Problem problem;

    problem.AddResidualBlock(new ceres::AutoDiffCostFunction<CostFunctor, 1, 1>(new CostFunctor), nullptr, &x);

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.BriefReport() << "\n";
    std::cout << "Final x = " << x << "\n";
    return 0;
}
```

#### 代价结构体
```
struct CostFunctor {
    template <typename T>
    bool operator()(const T* const x, T* residual) const {
        residual[0] = T(10.0) - x[0];
        return true;
    }
};
```

`template <typename T>` T为模板参数，支持多数据类型（double, Jet）  
Jet是ceres的自动微分类型

`bool operator()(const T* const x, T* residual) const`参数解释：  
`const T* const x`：指向优化参数的常量指针  
`T* residual`：输出残差的指针  
const的使用能够保证不修改对象状态

#### 初始化阶段
```
google::InitGoogleLogging(argv[0]);
double x = 0.0;
ceres::Problem problem;
```
`google::InitGoogleLogging(argv[0]);`用于初始化google日志系统，用于输出优化进度信息

#### 添加残差块
`problem.AddResidualBlock(new ceres::AutoDiffCostFunction<CostFunctor, 1, 1>(new CostFunctor), nullptr, &x);`

`AutoDiffCostFunction<CostFunctor, 1, 1>(new CostFunctor)`创建自动微分代价函数对象  
第一个'1'为残差维度，第二个'1'为参数块维度  
`nullptr`为标准的最小二乘损失函数，用于处理异常值，增加鲁棒性  
`&x`为要优化的参数指针

#### 配置求解器
```
ceres::Solver::Options options;
options.linear_solver_type = ceres::DENSE_QR;
options.minimizer_progress_to_stdout = true;
```

`options.linear_solver_type = ceres::DENSE_QR;`使用稠密QR分解  
`DENSE_QR`稠密矩阵（参数数量<1000）；QR分解（线性求解）  
`options.minimizer_progress_to_stdout = true;`迭代优化进度输出

#### 执行优化
```
ceres::Solver::Summary summary;
ceres::Solve(options, &problem, &summary);
```

### 曲线拟合
拟合曲线y = e ^ (a*x + b)

```
struct ExponentialResidual {
    ExponentialResidual(double x, double y) : x_(x), y_(y) {}
    
    template <typename T>
    bool operator()(const T* const ab, T* residual) const {
        residual[0] = T(y_) - exp(ab[0] * T(x_) + ab[1]);
        return true;
    }
    
    double x_, y_;
};

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    
    // 生成模拟数据
    std::vector<double> x_data, y_data;
    double a_true = 0.3, b_true = 0.1;
    std::default_random_engine gen;
    std::normal_distribution<double> noise(0.0, 0.2);
    
    for (int i = 0; i < 100; ++i) {
        double x = i / 100.0;
        x_data.push_back(x);
        y_data.push_back(exp(a_true * x + b_true) + noise(gen));
    }
    
    // 构建优化问题
    double ab[2] = {0.0, 0.0}; // 初始值
    
    ceres::Problem problem;
    for (int i = 0; i < x_data.size(); ++i) {
        problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<ExponentialResidual, 1, 2>(
                new ExponentialResidual(x_data[i], y_data[i])),
            nullptr, ab);
    }
    
    // 运行优化
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;
    
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    
    std::cout << summary.BriefReport() << std::endl;
    std::cout << "True a, b = " << a_true << ", " << b_true << std::endl;
    std::cout << "Estimated a, b = " << ab[0] << ", " << ab[1] << std::endl;
    
    return 0;
}
```

#### 残差结构体
```
struct ExponentialResidual {
    ExponentialResidual(double x, double y) : x_(x), y_(y) {}
    
    template <typename T>
    bool operator()(const T* const ab, T* residual) const {
        residual[0] = T(y_) - exp(ab[0] * T(x_) + ab[1]);
        return true;
    }
    
    double x_, y_;
};
```

`ExponentialResidual(double x, double y) : x_(x), y_(y) {}`存储一个数据点(x, y), 每个数据点对应一个残差项  
`x_(x)`将构造函数的参数x赋值给类的成员变量x_

#### 主函数部分
`std::normal_distribution<double> noise(0.0, 0.2);`噪声模型，噪声 ~ (0, 0.2^2)