#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

using namespace cv;
using namespace std;


bool  ROI_AddImage();
bool  LinearBlending();
bool  ROI_LinearBlending();
void   ShowHelpText();

int main(   )
{

	if(ROI_AddImage( )&& LinearBlending( )&&ROI_LinearBlending( ))
	{
		cout<<endl<<" ";
	}

	waitKey(0);
	return 0;
}

bool  ROI_AddImage()
{

	// ��1������ͼ��
	Mat srcImage1= imread("../dota_pa.jpg");
	Mat logoImage= imread("../dota_logo.jpg");
	if( !srcImage1.data ) { printf("��ȡsrcImage1����~�� \n"); return false; }
	if( !logoImage.data ) { printf("��ȡlogoImage����~�� \n"); return false; }

	// ��2������һ��Mat���Ͳ������趨ROI����
	Mat imageROI= srcImage1(Rect(200,250,logoImage.cols,logoImage.rows));

	// ��3��������ģ�������ǻҶ�ͼ��
	Mat mask= imread("../dota_logo.jpg",0);

	//��4������Ĥ������ROI
	logoImage.copyTo(imageROI,mask);

	// ��5����ʾ���?
	namedWindow("srcImage1");
	imshow("srcImage1",srcImage1);
	waitKey(0);

	return true;
}


bool  LinearBlending()
{
	//��0������һЩ�ֲ�����
	double alphaValue = 0.5; 
	double betaValue;
	Mat srcImage2, srcImage3, dstImage;

	// ��1����ȡͼ�� ( ����ͼƬ��Ϊͬ�������ͺͳߴ� )
	srcImage2 = imread("../mogu.jpg");
	srcImage3 = imread("../rain.jpg");

	if( !srcImage2.data ) { printf("��ȡsrcImage2���� \n"); return false; }
	if( !srcImage3.data ) { printf("��ȡsrcImage3���� \n"); return false; }

	// ��2������ͼ���ϼ�Ȩ����
	betaValue = ( 1.0 - alphaValue );
	addWeighted( srcImage2, alphaValue, srcImage3, betaValue, 0.0, dstImage);

	// ��3����ʾԭͼ����
	imshow( "srcImage2", srcImage2 );
	waitKey(0);
	imshow( "dstImage", dstImage );
	waitKey(0);

	return true;

}

bool  ROI_LinearBlending()
{

	//��1����ȡͼ��
	Mat srcImage4= imread("../dota_pa.jpg",1);
	Mat logoImage= imread("../dota_logo.jpg");

	if( !srcImage4.data ) { printf("��ȡsrcImage4����~�� \n"); return false; }
	if( !logoImage.data ) { printf("��ȡlogoImage����~�� \n"); return false; }

	//��2������һ��Mat���Ͳ������趨ROI����
	Mat imageROI;
	//����һ
	imageROI= srcImage4(Rect(200,250,logoImage.cols,logoImage.rows));
	//������
	//imageROI= srcImage4(Range(250,250+logoImage.rows),Range(200,200+logoImage.cols));

	//��3����logo�ӵ�ԭͼ��
	addWeighted(imageROI,0.5,logoImage,0.3,0.,imageROI);

	//��4����ʾ���?
	imshow("srcImage4",srcImage4);
	waitKey(0);

	return true;
}

