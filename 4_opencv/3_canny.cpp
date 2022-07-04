#include <opencv2/opencv.hpp>
#include<opencv2/imgproc/imgproc.hpp>

int main(){
	cv::Mat srcImage = cv::imread("../3_canny.jpg");
	cv::imshow("srcImage", srcImage);
	cv::Mat gray_image;
	cvtColor( srcImage, gray_image, cv::COLOR_BGR2GRAY );
    cv::Mat canny_image;
	blur( gray_image, canny_image, cv::Size(3,3) );
	Canny( canny_image, canny_image, 3, 9,3 );
	imshow("Canny", canny_image);
	cv::waitKey(0);
	return 0;
}