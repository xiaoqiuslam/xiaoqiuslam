#include <opencv2/opencv.hpp>

int main( ){
	cv::VideoCapture capture("../4_video_capture.avi");
	while(1){
		cv::Mat frame;
		capture>>frame;
		if (frame.empty()){
			break;
		}
		cv::imshow("frame",frame);
		cv::waitKey(30);
	}
	return 0;
}  
