#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

using namespace cv;
using namespace std;

// fuction statement
// 颜色空间转换
void colorSpaceConversionToGrayscale(const Mat& src);
void colorSpaceConversionToHSV(const Mat& src);

// 滤波
void applyAverageFilters(const Mat& src);
void applyGaussFilters(const Mat& src);

// 红色特征提取与边框处理
void extractRedFeatures(const Mat& src);

// 提取高亮颜色区域并进行图形学处理
void extractBrightRegions(const Mat& src);

// 图像绘制
void drawShapesAndText(Mat& image);

//图像处理
void imageRotate(const Mat& src);
void imageCut(const Mat& src);

int main() {
    // 读取图像
    string imagePath = "../resources/test_image.png";
    Mat image = imread(imagePath);
    
    if (image.empty()) {
        cerr << "无法加载图像: " << imagePath << endl;
        return -1;
    }
    
    cout << "图像加载成功！尺寸: " << image.cols << "x" << image.rows << endl;
    
    colorSpaceConversionToGrayscale(image);
    colorSpaceConversionToHSV(image);
    
    applyAverageFilters(image);
    applyGaussFilters(image);
    
    extractRedFeatures(image);
    
    extractBrightRegions(image);
    
    Mat drawImage = image.clone();
    drawShapesAndText(drawImage);
    imshow("Drawing Demo", drawImage);
    
    imageRotate(image);
    imageCut(image);
    
    waitKey(0);
    return 0;
}

// 1. 颜色空间转换
// 转换为灰度图
void colorSpaceConversionToGrayscale(const Mat& src) {
    Mat grayImage;
    cvtColor(src, grayImage, COLOR_BGR2GRAY);
    imshow("Gray Image", grayImage);
    imwrite("../resources/test_gray_image.png", grayImage);
    
}

// 转换为HSV图像
void colorSpaceConversionToHSV(const Mat& src) {
    Mat hsvImage;
    cvtColor(src, hsvImage, COLOR_BGR2HSV);
    imshow("HSV Image", hsvImage);
    imwrite("../resources/test_hsv_image.png", hsvImage);
}


// 2. 滤波
// 均值滤波
void applyAverageFilters(const Mat& src) {
    Mat blurImage;
    blur(src, blurImage, Size(5, 5));
    imshow("Mean Blur", blurImage);
    imwrite("../resources/test_mean_blur.png", blurImage);
}

// 高斯滤波
void applyGaussFilters(const Mat& src) {
    Mat gaussianImage;
    GaussianBlur(src, gaussianImage, Size(5, 5), 1.5);
    imshow("Gaussian Blur", gaussianImage);
    imwrite("../resources/test_gaussian_blur.png", gaussianImage);
}

// 3. 红色特征提取与边框处理
void extractRedFeatures(const Mat& src) {
    Mat hsvImage;
    cvtColor(src, hsvImage, COLOR_BGR2HSV);
    
    // 定义红色的HSV范围
    Mat mask1, mask2, redMask;
    inRange(hsvImage, Scalar(0, 0, 50), Scalar(15, 255, 255), mask1);
    inRange(hsvImage, Scalar(130, 0, 50), Scalar(180, 255, 255), mask2);
    redMask = mask1 | mask2;
    
    // 提取红色区域
    Mat redRegion;
    bitwise_and(src, src, redRegion, redMask);
    imshow("Red Region", redRegion);
    imwrite("../resources/test_red_region.png", redRegion);
    
    // 寻找轮廓
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(redMask, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    
    // 绘制结果
    Mat result = src.clone();
    for (size_t i = 0; i < contours.size(); i++) {
        // 计算轮廓面积
        double area = contourArea(contours[i]);
        if (area > 100) { // 只处理面积大于100的轮廓
            // 绘制轮廓
            drawContours(result, contours, (int)i, Scalar(0, 255, 0), 2);
            
            // 计算边界框
            Rect boundingBox = boundingRect(contours[i]);
            rectangle(result, boundingBox, Scalar(255, 0, 0), 2);
            
            // 显示面积
            // string areaText = "Area: " + to_string((int)area);
            // putText(result, areaText, Point(boundingBox.x, boundingBox.y - 10),
            //        FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1);
        }
    }
    
    imshow("Red Features Extraction", result);
    imwrite("../resources/test_red_features.png", result);
}

// 4. 提取高亮颜色区域并进行图形学处理
void extractBrightRegions(const Mat& src) {
    // 灰度化
    Mat gray;
    cvtColor(src, gray, COLOR_BGR2GRAY);
    
    // 二值化
    Mat binary;
    threshold(gray, binary, 188, 255, THRESH_BINARY);
    
    // 形态学操作 - 膨胀
    Mat dilated;
    Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
    dilate(binary, dilated, kernel);
    
    // 形态学操作 - 腐蚀
    Mat eroded;
    erode(dilated, eroded, kernel);
    
    // 漫水填充
    Mat floodFilled = eroded.clone();
    floodFill(floodFilled, Point(0, 0), Scalar(255));
    
    // 显示处理过程
    imshow("1. Gray", gray);
    imshow("2. Binary", binary);
    imshow("3. Dilated", dilated);
    imshow("4. Eroded", eroded);
    imshow("5. Flood Filled", floodFilled);
    
    // 保存结果
    imwrite("../resources/test_bright_gray.png", gray);
    imwrite("../resources/test_bright_binary.png", binary);
    imwrite("../resources/test_bright_dilated.png", dilated);
    imwrite("../resources/test_bright_eroded.png", eroded);
    imwrite("../resources/test_bright_floodfilled.png", floodFilled);
}

// 5. 图像绘制
void drawShapesAndText(Mat& image) {
    // 绘制圆形
    circle(image, Point(100, 100), 50, Scalar(0, 0, 255), 3);
    
    // 绘制矩形
    rectangle(image, Point(200, 50), Point(300, 150), Scalar(0, 255, 0), 2);
    
    // 绘制文字
    putText(image, "OpenCV Demo", Point(50, 200), 
           FONT_HERSHEY_SIMPLEX, 1.0, Scalar(255, 255, 0), 2);
    
    // 绘制线条
    line(image, Point(400, 50), Point(500, 150), Scalar(255, 0, 0), 2);
}

// 6. 图像变换
void imageRotate(const Mat& src) {
    // 图像旋转35度
    Point2f center(src.cols / 2.0f, src.rows / 2.0f);
    Mat rotationMatrix = getRotationMatrix2D(center, 35.0, 1.0);
    Mat rotatedImage;
    warpAffine(src, rotatedImage, rotationMatrix, src.size());
    imshow("Rotated 35 degrees", rotatedImage);
    imwrite("../resources/test_rotated_35.png", rotatedImage);
}

void imageCut(const Mat& src) {
    // 图像裁剪为原图的左上角1/4
    Rect cropRegion(0, 0, src.cols / 2, src.rows / 2);
    Mat croppedImage = src(cropRegion).clone();
    imshow("Cropped Top-Left 1/4", croppedImage);
    imwrite("../resources/test_cropped_quarter.png", croppedImage);
}

