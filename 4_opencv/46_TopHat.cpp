#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
using namespace cv;



//-----------------------------------【main( )函数】------------------------------------------
//		描述：控制台应用程序的入口函数，我们的程序从这里开始
//-----------------------------------------------------------------------------------------------
int main( )
{
	//载入原始图   
	Mat image = imread("../41_DilationEX.jpg");  //工程目录下应该有一张名为1.jpg的素材图
	//创建窗口   
	namedWindow("【原始图】顶帽运算");  
	namedWindow("【效果图】顶帽运算");  
	//显示原始图  
	imshow("【原始图】顶帽运算", image);  
	//定义核
	Mat element = getStructuringElement(MORPH_RECT, Size(15, 15));  
	//进行形态学操作
	morphologyEx(image, image, MORPH_TOPHAT, element);
	//显示效果图  
	imshow("【效果图】顶帽运算", image);  

	waitKey(0);  

	return 0;  
}
