//		程序描述：方框滤波boxFilter函数的使用示例程序
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp" 
#include "opencv2/imgproc/imgproc.hpp" 
using namespace cv; 
int main( )
{ 
	// 载入原图
	Mat image=imread("../31_boxFilter.jpg");

	//创建窗口
	namedWindow( "boxFilter1" );
	namedWindow( "boxFilter2");

	//显示原图
	imshow( "boxFilter1", image );

	//进行方框滤波操作
	Mat out; 
	boxFilter( image, out, -1,Size(5, 5)); 

	//显示效果图
	imshow( "boxFilter2" ,out );

	waitKey( 0 );     
} 