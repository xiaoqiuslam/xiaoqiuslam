
#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>


int main()
{
    cv::Mat imgLeft =  cv::imread("/media/wason/q/xiaoqiuslam/2_opencv_ros/a_opencv_cvmat/MH_01_easy_mav0_cam0_data_1403636579913555456.png", -1);
    cv::Mat imgRight =  cv::imread("/media/wason/q/xiaoqiuslam/2_opencv_ros/a_opencv_cvmat/MH_01_easy_mav0_cam1_data_1403636579913555456.png", -1);

    std::cout << "imgLeft.rows*1/2 " << imgLeft.rows*1/2 << std::endl;// 240
    std::cout << "imgLeft.rows " << imgLeft.rows << std::endl;// 480

    
    for (int row = imgLeft.rows*1/2;  row < imgLeft.rows;  row++) {
        // .at<>()返回的是一个值 
        // .ptr<>()返回的是一个地址
        // 遍历行 获取行首元素的地址
       uchar *imgLeft_r = imgLeft.ptr<uchar>(row);
       uchar *imgRight_r = imgRight.ptr<uchar>(row);
       //  遍历列  像素重置  之所以用指针是为了加速
        for (int col = 0; col < imgLeft.cols; col++) {
            imgLeft_r[col] = 0;
            imgRight_r[col] = 0;
        }
    }
    cv::imwrite("../imgLeft.png", imgLeft);
    cv::imwrite("../imgRight.png", imgRight);
}


