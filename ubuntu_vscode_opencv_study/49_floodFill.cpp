#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>  
using namespace cv;  



//-----------------------------------【main( )函数】--------------------------------------------  
//      描述：控制台应用程序的入口函数，我们的程序从这里开始  
//----------------------------------------------------------------------------------------------- 
int main( )
{    
	Mat src = imread("../49_floodFill.jpg");
	imshow("【原始图】",src);
	Rect ccomp;
	floodFill(src, Point(50,300), Scalar(155, 255,55), &ccomp, Scalar(20, 20, 20),Scalar(20, 20, 20));
	imshow("【效果图】",src);
	waitKey(0);
	return 0;    
}  