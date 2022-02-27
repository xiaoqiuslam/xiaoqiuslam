#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

using namespace std;
using namespace cv;

int main(){
    Mat image(6, 6, CV_8UC1);
    for (int i = 0; i < image.rows; i++)
    {
        for (int j = 0; j < image.cols; j++)
        {
            if (j == 3)
            {
                image.at<char>(i, j) = 255;
            }
        }
    }
    int step1 = (int)image.step1();
    cout << "(int)image.step1()= " << step1 << endl;
    int step2 = (int)image.step;
    cout << "(int)image.step()= " << step2 << endl;
    cv::namedWindow("image", CV_WINDOW_NORMAL);
    imshow("image", image);
    cv::waitKey(0);


    cv::Mat image_gray(9, 9, CV_8UC1, cv::Scalar(99));

    for(int i = 0; i < image_gray.rows; i++){
        uchar* data = image_gray.ptr<uchar>(i);
        for(int j = 0; j < image_gray.cols * image_gray.channels(); j++){
            cout << float(data[j]) << " ";
        }
        cout << endl;
    }
    int step3 = (int)image.step1();
    cout << "(int)image.step1()= " << step3 << endl;
    int step4 = (int)image.step;
    cout << "(int)image.step()= " << step4 << endl;
    cv::namedWindow("image_gray", CV_WINDOW_NORMAL);
    imshow("image_gray",image_gray);
    cv::waitKey(0);


    Mat I = Mat::eye(4, 4, CV_64F);
    I.at<double>(1,1) = CV_PI;
    cout << "\nI = " << I << ";\n" << endl;

}