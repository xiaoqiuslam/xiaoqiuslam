#include<opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main(){
    cv::Mat image;
    image = imread("/home/q/slambook2/ch7/1.png");

    int m00 = 0;
    int m10 = 0;
    int m01 = 0;

    // 计算图像像素点灰度值的和
    for (int i = 30; i < 60; i++){
        for (int j = 150; j < 180; j++){
            m00 += image.at<uchar>(i, j);
        }
    }

    // 计算图像像素点坐标与像素点灰度值的乘积
    for (int i = 30; i < 60; i++){
        for (int j = 150; j < 180; j++){
            m10 += i * image.at<uchar>(i, j);
            m01 += j * image.at<uchar>(i, j);
        }
    }

    cv::rectangle(image, cv::Rect(30, 150, 30, 30), 0, 1, 8);

    // 计算质心坐标 质心(一阶矩)
    Point2d point;
    point.x = static_cast<double>(m10) / m00;
    point.y = static_cast<double>(m01) / m00;

    circle(image, point, 3, Scalar(66), -1);
    cout << "质心坐标："<< point << endl;
    cv::imwrite("../质心.png",image);
}


