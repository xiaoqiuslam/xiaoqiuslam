#include <opencv2/opencv.hpp>
int main(){
	cv::VideoCapture capture(0);
	while(1){
		cv::Mat frame;
		capture>>frame;
		cv::imshow("frame",frame);
		cv::waitKey(30);
	}  
	return 0;     
}  
