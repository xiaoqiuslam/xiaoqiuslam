#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
using namespace cv;


#define WINDOW_NAME1 "��ԭʼͼƬ��"        //Ϊ���ڱ��ⶨ��ĺ�
#define WINDOW_NAME2 "��ƥ�䴰�ڡ�"        //Ϊ���ڱ��ⶨ��ĺ� 

Mat g_srcImage; Mat g_templateImage; Mat g_resultImage;
int g_nMatchMethod;
int g_nMaxTrackbarNum = 5;

//-----------------------------------��ȫ�ֺ����������֡�--------------------------------------
//          ������ȫ�ֺ���������
//-----------------------------------------------------------------------------------------------
void on_Matching( int, void* );
static void ShowHelpText( );


//-----------------------------------��main( )������--------------------------------------------
//          ����������̨Ӧ�ó������ں��������ǵĳ�������￪ʼִ��
//-----------------------------------------------------------------------------------------------
int main(  )
{
	//��0���ı�console������ɫ
	system("color 1F"); 

	//��0����ʾ��������
	ShowHelpText();

	//��1������ԭͼ���ģ���
	g_srcImage = imread( "../84_1_matchTemplate.jpg", 1 );
	g_templateImage = imread( "../84_2_matchTemplate.jpg", 1 );

	//��2����������
	namedWindow( WINDOW_NAME1, WINDOW_AUTOSIZE );
	namedWindow( WINDOW_NAME2, WINDOW_AUTOSIZE );

	//��3������������������һ�γ�ʼ��
	createTrackbar( "����", WINDOW_NAME1, &g_nMatchMethod, g_nMaxTrackbarNum, on_Matching );
	on_Matching( 0, 0 );

	waitKey(0);
	return 0;

}

//-----------------------------------��on_Matching( )������--------------------------------
//          �������ص�����
//-------------------------------------------------------------------------------------------
void on_Matching( int, void* )
{
	//��1�����ֲ�������ʼ��
	Mat srcImage;
	g_srcImage.copyTo( srcImage );

	//��2����ʼ�����ڽ������ľ���
	int resultImage_rows = g_srcImage.rows - g_templateImage.rows + 1;
	int resultImage_cols =  g_srcImage.cols - g_templateImage.cols + 1;
	g_resultImage.create(resultImage_rows,resultImage_cols, CV_32FC1);

	//��3������ƥ��ͱ�׼��
	matchTemplate( g_srcImage, g_templateImage, g_resultImage, g_nMatchMethod );
	normalize( g_resultImage, g_resultImage, 0, 1, NORM_MINMAX, -1, Mat() );

	//��4��ͨ������ minMaxLoc ��λ��ƥ���λ��
	double minValue; double maxValue; Point minLocation; Point maxLocation;
	Point matchLocation;
	minMaxLoc( g_resultImage, &minValue, &maxValue, &minLocation, &maxLocation, Mat() );

	//��5�����ڷ��� SQDIFF �� SQDIFF_NORMED, ԽС����ֵ���Ÿ��ߵ�ƥ����. ������ķ���, ��ֵԽ��ƥ��Ч��Խ��
	//�˾�����OpenCV2��Ϊ��
	//if( g_nMatchMethod  == CV_TM_SQDIFF || g_nMatchMethod == CV_TM_SQDIFF_NORMED )
	//�˾�����OpenCV3��Ϊ��
	if( g_nMatchMethod  == TM_SQDIFF || g_nMatchMethod == TM_SQDIFF_NORMED )
	{ matchLocation = minLocation; }
	else
	{ matchLocation = maxLocation; }

	//��6�����Ƴ����Σ�����ʾ���ս��
	rectangle( srcImage, matchLocation, Point( matchLocation.x + g_templateImage.cols , matchLocation.y + g_templateImage.rows ), Scalar(0,0,255), 2, 8, 0 );
	rectangle( g_resultImage, matchLocation, Point( matchLocation.x + g_templateImage.cols , matchLocation.y + g_templateImage.rows ), Scalar(0,0,255), 2, 8, 0 );

	imshow( WINDOW_NAME1, srcImage );
	imshow( WINDOW_NAME2, g_resultImage );

}



//-----------------------------------��ShowHelpText( )������----------------------------------
//          ���������һЩ������Ϣ
//----------------------------------------------------------------------------------------------
static void ShowHelpText()
{
	//�����ӭ��Ϣ��OpenCV�汾
	printf("\n\n\t\t\t�ǳ���л����OpenCV3������š�һ�飡\n");
	printf("\n\n\t\t\t��Ϊ����OpenCV3��ĵ�84������ʾ������\n");
	printf("\n\n\t\t\t   ��ǰʹ�õ�OpenCV�汾Ϊ��" CV_VERSION );
	printf("\n\n  ----------------------------------------------------------------------------\n");
	//���һЩ������Ϣ
	printf("\t��ӭ������ģ��ƥ�䡿ʾ������~\n"); 
	printf("\n\n\t������������۲�ͼ��Ч��\n\n");
	printf(  "\n\t��������Ӧ�ķ�����ֵ˵��: \n\n" 
		"\t\t������0��- ƽ����ƥ�䷨(SQDIFF)\n" 
		"\t\t������1��- ��һ��ƽ����ƥ�䷨(SQDIFF NORMED)\n" 
		"\t\t������2��- ���ƥ�䷨(TM CCORR)\n" 
		"\t\t������3��- ��һ�����ƥ�䷨(TM CCORR NORMED)\n" 
		"\t\t������4��- ���ϵ��ƥ�䷨(TM COEFF)\n" 
		"\t\t������5��- ��һ�����ϵ��ƥ�䷨(TM COEFF NORMED)\n" );  
}
