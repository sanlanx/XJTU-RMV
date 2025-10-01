#include <iostream>
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include <ceres/ceres.h>
#include <glog/logging.h>

using namespace std;
using namespace cv;

// 弹道模型参数结构
struct TrajectoryParams {
    double x0;
    double y0;
    double vx0;
    double vy0;
    double g;
    double k;
};

// 弹道模型函数
void trajectory_model(double t, const TrajectoryParams& params, double& x, double& y) {
    double dt = t;  // t0 = 0
    double exp_kt = exp(-params.k * dt);
    
    x = params.x0 + (params.vx0 / params.k) * (1 - exp_kt);
    y = params.y0 + ((params.vy0 + params.g / params.k) / params.k) * (1 - exp_kt) - (params.g / params.k) * dt;
}

// Ceres残差函数
struct TrajectoryResidual {
    TrajectoryResidual(double t, double x_observed, double y_observed)
        : t_(t), x_observed_(x_observed), y_observed_(y_observed) {}
    
    template <typename T>
    bool operator()(const T* const params, T* residual) const {
        // 解包参数
        const T& x0 = params[0];
        const T& y0 = params[1];
        const T& vx0 = params[2];
        const T& vy0 = params[3];
        const T& g = params[4];
        const T& k = params[5];
        
        // 计算模型预测的位置
        T dt = T(t_);
        T exp_kt = exp(-k * dt);
        
        T x_predicted = x0 + (vx0 / k) * (T(1.0) - exp_kt);
        T y_predicted = y0 + ((vy0 + g / k) / k) * (T(1.0) - exp_kt) - (g / k) * dt;
        
        // 计算残差（x和y方向的误差）
        residual[0] = x_predicted - T(x_observed_);
        residual[1] = y_predicted - T(y_observed_);
        
        return true;
    }
    
private:
    double t_;
    double x_observed_;
    double y_observed_;
};

// 小球检测函数
bool detect_ball(const Mat& frame, Point2d& ball_center, Point2d& draw_center) {
    // 灰度转换与二值化处理
    Mat grayImage, binary;
    cvtColor(frame, grayImage, COLOR_BGR2GRAY);
    threshold(grayImage, binary, 128, 255, THRESH_BINARY);
    
    Mat whiteMask = binary; 
    
    // 形态学操作（去除噪声）
    Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5));
    morphologyEx(whiteMask, whiteMask, MORPH_CLOSE, kernel);
    morphologyEx(whiteMask, whiteMask, MORPH_OPEN, kernel);
    
    // 查找轮廓
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(whiteMask, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    
    if (contours.empty() || contourArea(contours[0]) < 10) { // 面积阈值，避免噪声
        return false;
    }
    
    // 计算轮廓的矩和质心
    Moments m = moments(contours[0]);
    double img_x = m.m10 / m.m00;
    double img_y = m.m01 / m.m00;
    
    ball_center.x = img_x;
    ball_center.y = frame.rows - img_y;

    draw_center.x = img_x;
    draw_center.y = img_y;

    return true;
}

int main(int argc, char** argv) {
    // 初始化Google Logging
    google::InitGoogleLogging(argv[0]);
    
    // 打开视频文件
    VideoCapture capture("../videos/video.mp4");
    if (!capture.isOpened()) {
        cout << "无法打开视频文件！" << endl;
        return -1;
    }
    
    // 获取视频信息
    double fps = capture.get(CAP_PROP_FPS);
    int total_frames = capture.get(CAP_PROP_FRAME_COUNT);
    
    cout << "视频FPS: " << fps << endl;
    cout << "总帧数: " << total_frames << endl;
    
    // 存储检测到的小球轨迹
    vector<double> time_points;
    vector<double> x_positions;
    vector<double> y_positions;
    
    Mat frame;
    int frame_count = 0;
    
    // 逐帧处理视频
    while (capture.read(frame)) {
        if (frame.empty()) {
            break;
        }
        
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
        
        // 可选：显示处理后的帧
        imshow("Ball Detection", frame);
        if (waitKey(1) == 27) { // ESC键退出
            break;
        }
        
        frame_count++;
    }
    
    capture.release();
    destroyAllWindows();
    
    cout << "检测到 " << time_points.size() << " 个有效数据点" << endl;
    
    if (time_points.size() < 10) {
        cout << "数据点太少，无法进行拟合！" << endl;
        return -1;
    }
    
    // 初始化参数（根据物理常识设置合理的初始值）
    double params[6] = {x_positions[0], y_positions[0], 250.0, 350.0, 500.0, 0.066};
    
    // 创建Ceres问题
    ceres::Problem problem;
    
    // 添加残差块
    for (size_t i = 0; i < time_points.size(); i++) {
        ceres::CostFunction* cost_function = 
            new ceres::AutoDiffCostFunction<TrajectoryResidual, 2, 6>(
                new TrajectoryResidual(time_points[i], x_positions[i], y_positions[i]));
        
        problem.AddResidualBlock(cost_function, NULL, params);
    }
    
    // 配置求解器选项
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;
    // options.max_num_iterations = 100;
    // options.function_tolerance = 1e-6;
    
    // 求解
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    
    // 输出结果
    cout << summary.FullReport() << endl;
    cout << "拟合结果：" << endl;
    cout << "x0 = " << params[0] << " px" << endl;
    cout << "y0 = " << params[1] << " px" << endl;
    cout << "vx0 = " << params[2] << " px/s" << endl;
    cout << "vy0 = " << params[3] << " px/s" << endl;
    cout << "g = " << params[4] << " px/s²" << endl;
    cout << "k = " << params[5] << " 1/s" << endl;
    
    // 计算平均误差
    double total_x_error = 0.0;
    double total_y_error = 0.0;
    double total_relative_x_error = 0.0;
    double total_relative_y_error = 0.0;
    
    TrajectoryParams fitted_params = {params[0], params[1], params[2], params[3], params[4], params[5]};
    
    for (size_t i = 0; i < time_points.size(); i++) {
        double x_pred, y_pred;
        trajectory_model(time_points[i], fitted_params, x_pred, y_pred);
        
        double x_error = abs(x_pred - x_positions[i]);
        double y_error = abs(y_pred - y_positions[i]);
        
        total_x_error += x_error;
        total_y_error += y_error;
        total_relative_x_error += (x_error / x_positions[i] * 100);
        total_relative_y_error += (y_error / y_positions[i] * 100);
    }
    
    double avg_x_error = total_x_error / time_points.size();
    double avg_y_error = total_y_error / time_points.size();
    double avg_relative_x_error = total_relative_x_error / time_points.size();
    double avg_relative_y_error = total_relative_y_error / time_points.size();
    
    cout << "\n平均误差统计：" << endl;
    cout << "X方向平均绝对误差: " << avg_x_error << " px" << endl;
    cout << "Y方向平均绝对误差: " << avg_y_error << " px" << endl;
    cout << "X方向平均相对误差: " << avg_relative_x_error << "%" << endl;
    cout << "Y方向平均相对误差: " << avg_relative_y_error << "%" << endl;
    
    return 0;
}