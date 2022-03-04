#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
using namespace cv;

int main( )
{
	//载入原始图   
	Mat srcImage = imread("../51_resize.jpg");  //工程目录下应该有一张名为1.jpg的素材图
	Mat tmpImage,dstImage;//临时变量和目标图的定义
	tmpImage=srcImage;//将原始图赋给临时变量

	//显示原始图  
	imshow("【原始图】", srcImage);  
	//进行向下取样操作
	pyrDown( tmpImage, dstImage, Size( tmpImage.cols/2, tmpImage.rows/2 ) );
	//显示效果图  
	imshow("【效果图】", dstImage);  

	waitKey(0);  

	return 0;  
}
