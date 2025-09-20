#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>

using namespace cv;
using namespace std;

int main() {
    // 1. 读取图像
    Mat image = imread("../resources/test_image_2.png"); // 替换为你的图片路径
    if (image.empty()) {
        cout << "无法加载图像！" << endl;
        return -1;
    }

    // 2. 转换为HSV颜色空间（更适合颜色识别）
    Mat hsvImage;
    cvtColor(image, hsvImage, COLOR_BGR2HSV);

    // 3. 定义蓝色的HSV范围（高亮蓝色）
    // HSV中蓝色的范围：Hue: 90-130, Saturation: 高饱和度, Value: 高亮度
    Scalar lowerBlue = Scalar(95, 130, 230);   // 下限
    Scalar upperBlue = Scalar(110, 255, 255);  // 上限

    // 4. 创建蓝色掩码
    Mat blueMask;
    inRange(hsvImage, lowerBlue, upperBlue, blueMask);

    // 5. 形态学操作（可选，用于去除噪声和填充空洞）
    Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5));
    morphologyEx(blueMask, blueMask, MORPH_CLOSE, kernel);
    morphologyEx(blueMask, blueMask, MORPH_OPEN, kernel);
    dilate(blueMask, blueMask, kernel);

    // 6. 查找轮廓
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(blueMask, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    // 7. 收集所有符合条件的矩形
    vector<Rect> validRects;
    Mat result = image.clone();
    
    for (size_t i = 0; i < contours.size(); i++) {
        double area = contourArea(contours[i]);
        if (area > 500 && area < 1000) {
            Rect rect = boundingRect(contours[i]);
            float aspectRatio = (float)rect.width / rect.height;

            if (aspectRatio < 0.6) {
                validRects.push_back(rect);
                // 可选：绘制原始小矩形（浅色）
                // rectangle(result, rect, Scalar(0, 255, 0), 1);
            }
        }
    }

    // 8. 合并所有矩形为一个大矩形
    if (!validRects.empty()) {
        // 计算合并矩形的边界
        int minX = validRects[0].x;
        int minY = validRects[0].y;
        int maxX = validRects[0].x + validRects[0].width;
        int maxY = validRects[0].y + validRects[0].height;

        for (size_t i = 1; i < validRects.size(); i++) {
            minX = min(minX, validRects[i].x);
            minY = min(minY, validRects[i].y);
            maxX = max(maxX, validRects[i].x + validRects[i].width);
            maxY = max(maxY, validRects[i].y + validRects[i].height);
        }

        // 创建合并后的矩形
        Rect mergedRect(minX, minY, maxX - minX, maxY - minY);
        
        // 绘制合并后的大矩形（粗边框）
        rectangle(result, mergedRect, Scalar(0, 0, 255), 3);
        
        // 可选：显示合并矩形的信息
        string text = "Merged Rect: " + to_string(mergedRect.width) + "x" + to_string(mergedRect.height);
        putText(result, text, Point(mergedRect.x, mergedRect.y - 10), 
                FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 0, 255), 2);
    }

    // 9. 显示结果
    imshow("原始图像", image);
    //imshow("蓝色掩码", blueMask);
    imshow("检测结果", result);
    imwrite("../resources/dealed_image_2.png", result);

    waitKey(0);
    return 0;
}