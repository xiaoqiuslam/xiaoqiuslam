#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

int main(){
    cv::Mat srcImage = cv::imread("../39_erode.jpg");
    cv::imshow("srcImage", srcImage);
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(15, 15));
    cv::Mat erode_Image;
    cv::erode(srcImage, erode_Image, element);
    cv::imshow("erode", erode_Image);
    cv::waitKey(0);

    cv::Mat blur_Image;
    cv::blur( srcImage, blur_Image, cv::Size(7, 7));
    cv::imshow( "blur" ,blur_Image );
    cv::waitKey(0);
	return 0;
}