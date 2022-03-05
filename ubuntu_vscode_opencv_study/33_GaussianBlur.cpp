
// GaussianBlur

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp" 
#include "opencv2/imgproc/imgproc.hpp" 
using namespace cv; 

int main( )
{ 
	// 载入原图
	Mat image=imread("../33_GaussianBlur.jpg");
	//创建窗口
	namedWindow( "image" );
	namedWindow( "out");

	//显示原图
	imshow( "image", image );

	//进行高斯滤波操作
	Mat out; 
	GaussianBlur( image, out, Size( 5, 5 ), 0, 0 ); 

	//显示效果图
	imshow( "out" ,out );

	waitKey( 0 );     
} 