//		程序描述：用morphologyEx进行图像膨胀
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
using namespace cv;

int main( )
{
	//载入原始图   
	Mat image = imread("../41_DilationEX.jpg");  //工程目录下应该有一张名为1.jpg的素材图
	//创建窗口   
	namedWindow("【原始图】膨胀");  
	namedWindow("【效果图】膨胀");  
	//显示原始图  
	imshow("【原始图】膨胀", image);  
	//定义核
	Mat element = getStructuringElement(MORPH_RECT, Size(15, 15));  
	//进行形态学操作
	morphologyEx(image, image, MORPH_DILATE, element);
	//显示效果图  
	imshow("【效果图】膨胀", image);  

	waitKey(0);  

	return 0;  
}
