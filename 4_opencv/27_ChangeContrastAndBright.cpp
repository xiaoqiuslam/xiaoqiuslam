#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>

using namespace std;
using namespace cv;

static void ContrastAndBright(int, void *);
void   ShowHelpText();

int g_nContrastValue; //�Աȶ�ֵ
int g_nBrightValue;  //����ֵ
Mat g_srcImage,g_dstImage;
int main(   )
{
	ShowHelpText();
	g_srcImage = imread( "../27_ChangeContrastAndBright.jpg");
	if( !g_srcImage.data ) { printf("��ȡg_srcImageͼƬ����~�� \n"); return false; }
	g_dstImage = Mat::zeros( g_srcImage.size(), g_srcImage.type() );

	//�趨�ԱȶȺ����ȵĳ�ֵ
	g_nContrastValue=80;
	g_nBrightValue=80;

	//��������
	namedWindow("1", 1);

	//�����켣��
	createTrackbar("�Աȶȣ�", "��Ч��ͼ���ڡ�",&g_nContrastValue, 300,ContrastAndBright );
	createTrackbar("��   �ȣ�", "��Ч��ͼ���ڡ�",&g_nBrightValue, 200,ContrastAndBright );

	//���ûص�����
	ContrastAndBright(g_nContrastValue,0);
	ContrastAndBright(g_nBrightValue,0);

	//���һЩ�������?
	cout<<endl<<"\t���гɹ���������������۲�ͼ��Ч��\n\n"
		<<"\t���¡�q����ʱ�������˳�\n";

	//���¡�q����ʱ�������˳�
	while(char(waitKey(1)) != 'q') {}
	return 0;
}




//-----------------------------------��ShowHelpText( )������----------------------------------
//		 ���������һЩ�������?
//----------------------------------------------------------------------------------------------
void ShowHelpText()
{
	//�����ӭ��Ϣ��OpenCV�汾
	printf("\n\n\t\t\t�ǳ���л����OpenCV3������š�һ�飡\n");
	printf("\n\n\t\t\t��Ϊ����OpenCV3��ĵ�?27������ʾ������\n");
	printf("\n\n\t\t\t   ��ǰʹ�õ�OpenCV�汾Ϊ��" CV_VERSION );
	printf("\n\n  ----------------------------------------------------------------------------\n");
}


//-----------------------------��ContrastAndBright( )������------------------------------------
//	�������ı�ͼ��ԱȶȺ�����ֵ�Ļص�����?
//-----------------------------------------------------------------------------------------------
static void ContrastAndBright(int, void *)
{

	// ��������
	namedWindow("0", 1);

	// ����forѭ����ִ������ g_dstImage(i,j) = a*g_srcImage(i,j) + b
	for( int y = 0; y < g_srcImage.rows; y++ )
	{
		for( int x = 0; x < g_srcImage.cols; x++ )
		{
			for( int c = 0; c < 3; c++ )
			{
				g_dstImage.at<Vec3b>(y,x)[c] = saturate_cast<uchar>( (g_nContrastValue*0.01)*( g_srcImage.at<Vec3b>(y,x)[c] ) + g_nBrightValue );
			}
		}
	}

	// ��ʾͼ��
	imshow("g_srcImage", g_srcImage);
	imshow("g_dstImage", g_dstImage);
}


