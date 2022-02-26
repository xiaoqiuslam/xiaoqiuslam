#include<opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

Point2d First_moment_calculation_centroid(Mat Img);
Point2d Second_moment_calculation_centroid(Mat Img);

int main()
{
    Mat Img(300, 300, CV_8UC1);
    for (int i = 0; i < Img.rows; i++)
    {
        for (int j = 0; j < Img.cols; j++)
        {
            Img.at<char>(i, j) = 99;
        }
    }
    imshow("image", Img);
    cv::waitKey(0);

    cv::Point p(150, 150);
    circle(Img, p, 104, 33,-1);
    imshow("image", Img);
    cv::waitKey(0);

//    cv::Rect rect = cv::Rect(1, 1, 298, 298);
//    cv::rectangle(Img, rect, 0, -1, 8);
//    imshow("image", Img);
//    cv::waitKey(0);

    Point2d point1;

    point1 = First_moment_calculation_centroid(Img);

    cout << "质心坐标："<< point1 << endl;

    circle(Img, point1, 2, Scalar(66), -1);

    imshow("image", Img);
    cv::waitKey(0);

    Point2d point_2;
    point_2 = Second_moment_calculation_centroid(Img);
    cout << "质心坐标："<< point_2 << endl;

    circle(Img, point_2, 2, Scalar(88), -1);

    imshow("image", Img);
    cv::waitKey(0);
}

// 质心(一阶矩)
Point2d First_moment_calculation_centroid(Mat Img){
    int AllPointValue = 0;
    int x_AllPointValue = 0;
    int y_AllPointValue = 0;
    Point2d point;
    // 所有像素灰度值之和
    for (int i = 0; i < Img.rows; i++){
        for (int j = 0; j < Img.cols; j++){
//            cout << "灰度值: " << float(Img.at<uchar>(i, j)) << endl;
            AllPointValue += Img.at<uchar>(i, j);
        }
    }
    // 计算图像点所有点灰度值与其坐标的乘积之和
    for (int i = 0; i < Img.rows; i++){
        for (int j = 0; j < Img.cols; j++){
            x_AllPointValue += Img.at<uchar>(i, j) * j;
            cout << "pixel: " << float(Img.at<uchar>(i, j)) << " j: " <<  j << endl;
            y_AllPointValue += Img.at<uchar>(i, j) * i;
            cout << "pixel: " << float(Img.at<uchar>(i, j)) << " i: " <<  i << endl;
        }
    }
    // 计算质心坐标
    point.x = static_cast<double>(x_AllPointValue) / AllPointValue;
    point.y = static_cast<double>(y_AllPointValue) / AllPointValue;

    return point;
}

// 质心(二阶矩)
Point2d Second_moment_calculation_centroid(Mat Img){
    unsigned long long int AllPointValue = 0;
    unsigned long long int x_AllPointValue = 0;
    unsigned long long int y_AllPointValue = 0;
    Point2d point;
    for (int i = 0; i < Img.rows; i++){
        for (int j = 0; j < Img.cols; j++){
            AllPointValue += Img.at<uchar>(i, j);
        }
    }
    for (int i = 0; i < Img.rows; i++){
        for (int j = 0; j < Img.cols; j++){
            x_AllPointValue += static_cast<long long int>(Img.at<uchar>(i, j)) * j * j;
            y_AllPointValue += static_cast<long long int>(Img.at<uchar>(i, j)) * i * i;
        }
    }

    //计算质心坐标并存储
    point.x = sqrt(static_cast<double>(x_AllPointValue) / AllPointValue);
    point.y = sqrt(static_cast<double>(y_AllPointValue) / AllPointValue);

    return point;
}