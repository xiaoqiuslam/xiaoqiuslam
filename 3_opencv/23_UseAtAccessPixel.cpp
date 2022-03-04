#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>  
#include <iostream>  
using namespace std;  
using namespace cv;  

void colorReduce(Mat& inputImage, Mat& outputImage, int div);


int main( )
{  
	Mat srcImage = imread("../1_read.jpg");
	imshow("srcImage",srcImage);  
	waitKey(0);  

	Mat dstImage;
	dstImage.create(srcImage.rows,srcImage.cols,srcImage.type());//Ч��ͼ�Ĵ�С��������ԭͼƬ��ͬ 


	double time0 = static_cast<double>(getTickCount());

	colorReduce(srcImage,dstImage,32);

	time0 = ((double)getTickCount() - time0)/getTickFrequency();
	cout<<"�˷�������ʱ��Ϊ�� "<<time0<<"��"<<endl;  //�������ʱ��?

	//��6����ʾЧ��ͼ
	imshow("dstImage",dstImage);  
	waitKey(0);  
}  


//----------------------------------��colorReduce( )������-------------------------------
//          ������ʹ�á���̬��ַ�������at�������汾����ɫ�ռ���������
//----------------------------------------------------------------------------------------------
void colorReduce(Mat& inputImage, Mat& outputImage, int div)  
{  
	//����׼��
	outputImage = inputImage.clone();  //����ʵ�ε���ʱ����
	int rowNumber = outputImage.rows;  //����
	int colNumber = outputImage.cols;  //����

	//��ȡ��ɫͼ������
	for(int i = 0;i < rowNumber;i++)  
	{  
		for(int j = 0;j < colNumber;j++)  
		{  	
			// ------------------------����ʼ����ÿ�����ء�--------------------
			outputImage.at<Vec3b>(i,j)[0] =  outputImage.at<Vec3b>(i,j)[0]/div*div + div/2;  //��ɫͨ��
			outputImage.at<Vec3b>(i,j)[1] =  outputImage.at<Vec3b>(i,j)[1]/div*div + div/2;  //��ɫͨ��
			outputImage.at<Vec3b>(i,j)[2] =  outputImage.at<Vec3b>(i,j)[2]/div*div + div/2;  //����ͨ��
			// -------------------------������������----------------------------
		}  // �д�������     
	}  
}  






