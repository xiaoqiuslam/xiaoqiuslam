//		程序描述：均值滤波blur函数的使用示例程序
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp" 
using namespace cv; 

int main( )
{ 
	//【1】载入原始图
	Mat srcImage=imread("../32_blur.jpg");

	//【2】显示原始图
	imshow( "srcImage", srcImage );

	//【3】进行均值滤波操作
	Mat dstImage; 
	blur( srcImage, dstImage, Size(7, 7)); 

	//【4】显示效果图
	imshow( "dstImage" ,dstImage );

	waitKey( 0 );     
} 