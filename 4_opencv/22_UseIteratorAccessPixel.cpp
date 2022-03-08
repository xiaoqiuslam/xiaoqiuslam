

#include <opencv2/core/core.hpp>  
#include <opencv2/highgui/highgui.hpp>  
#include <iostream>  
using namespace std;  
using namespace cv;  



void colorReduce(Mat& inputImage, Mat& outputImage, int div);  



int main( )  
{  
	//【1】创建原始图并显示
	Mat srcImage = imread("../1_read.jpg");
	imshow("srcImage",srcImage);  
	waitKey(0);  

	//【2】按原始图的参数规格来创建创建效果图
	Mat dstImage;
	dstImage.create(srcImage.rows,srcImage.cols,srcImage.type());//效果图的大小、类型与原图片相同 


	//【3】记录起始时间
	double time0 = static_cast<double>(getTickCount());  

	//【4】调用颜色空间缩减函数
	colorReduce(srcImage,dstImage,32);  

	//【5】计算运行时间并输出
	time0 = ((double)getTickCount() - time0)/getTickFrequency();
	cout<<"此方法运行时间为： "<<time0<<"秒"<<endl;  //输出运行时间

	//【6】显示效果图
	imshow("dstImage",dstImage);  
	waitKey(0);  
}  




//-------------------------------------【colorReduce( )函数】-----------------------------
//		描述：使用【迭代器】方法版的颜色空间缩减函数
//----------------------------------------------------------------------------------------------
void colorReduce(Mat& inputImage, Mat& outputImage, int div)  
{  
	//参数准备
	outputImage = inputImage.clone();  //拷贝实参到临时变量
	//获取迭代器
	Mat_<Vec3b>::iterator it = outputImage.begin<Vec3b>();  //初始位置的迭代器
	Mat_<Vec3b>::iterator itend = outputImage.end<Vec3b>();  //终止位置的迭代器

	//存取彩色图像像素
	for(;it != itend;++it)  
	{  
		// ------------------------【开始处理每个像素】--------------------
		(*it)[0] = (*it)[0]/div*div + div/2;  
		(*it)[1] = (*it)[1]/div*div + div/2;  
		(*it)[2] = (*it)[2]/div*div + div/2;  
		// ------------------------【处理结束】----------------------------
	}  
}  




