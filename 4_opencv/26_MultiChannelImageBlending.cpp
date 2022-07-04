                                                                                    
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

using namespace cv;
using namespace std;


bool  MultiChannelBlending();


int main(   )
{


	if(MultiChannelBlending( ))
	{
		cout<<endl<<"\n";
	}

	waitKey(0);
	return 0;
}








bool  MultiChannelBlending()
{
	//��0��������ر���
	Mat srcImage;
	Mat logoImage;
	vector<Mat> channels;
	Mat  imageBlueChannel;

	//=================����ɫͨ�����֡�=================
	//	��������ͨ�����-��ɫ��������
	//============================================

	// ��1������ͼƬ
	logoImage= imread("../dota_logo.jpg",0);
	srcImage= imread("../dota_jugg.jpg");

	if( !logoImage.data ) { printf("Oh��no����ȡlogoImage����~�� \n"); return false; }
	if( !srcImage.data ) { printf("Oh��no����ȡsrcImage����~�� \n"); return false; }

	//��2����һ��3ͨ��ͼ��ת����3����ͨ��ͼ��
	split(srcImage,channels);//����ɫ��ͨ��

	//��3����ԭͼ����ɫͨ�����÷��ظ�imageBlueChannel��ע�������ã��൱�����ߵȼۣ��޸�����һ����һ�����ű�
	imageBlueChannel= channels.at(0);
	//��4����ԭͼ����ɫͨ���ģ�500,250�����괦���·���һ�������logoͼ���м�Ȩ���������õ��Ļ�Ͻ���浽imageBlueChannel��
	addWeighted(imageBlueChannel(Rect(500,250,logoImage.cols,logoImage.rows)),1.0,
		logoImage,0.5,0,imageBlueChannel(Rect(500,250,logoImage.cols,logoImage.rows)));

	//��5����������ͨ�����ºϲ���һ����ͨ��
	merge(channels,srcImage);

	//��6����ʾЧ��ͼ
	namedWindow("1srcImage");
	imshow("1srcImage",srcImage);


	//=================����ɫͨ�����֡�=================
	//	��������ͨ�����-��ɫ��������
	//============================================

	//��0��������ر���
	Mat  imageGreenChannel;

	//��1�����¶���ͼƬ
	logoImage= imread("../dota_logo.jpg",0);
	srcImage= imread("../dota_jugg.jpg");

	if( !logoImage.data ) { printf("��ȡlogoImage����~�� \n"); return false; }
	if( !srcImage.data ) { printf("��ȡsrcImage����~�� \n"); return false; }

	//��2����һ����ͨ��ͼ��ת����������ͨ��ͼ��
	split(srcImage,channels);//����ɫ��ͨ��

	//��3����ԭͼ����ɫͨ�������÷��ظ�imageBlueChannel��ע�������ã��൱�����ߵȼۣ��޸�����һ����һ�����ű�
	imageGreenChannel= channels.at(1);
	//��4����ԭͼ����ɫͨ���ģ�500,250�����괦���·���һ�������logoͼ���м�Ȩ���������õ��Ļ�Ͻ���浽imageGreenChannel��
	addWeighted(imageGreenChannel(Rect(500,250,logoImage.cols,logoImage.rows)),1.0,
		logoImage,0.5,0.,imageGreenChannel(Rect(500,250,logoImage.cols,logoImage.rows)));

	//��5�������������ĵ�ͨ�����ºϲ���һ����ͨ��
	merge(channels,srcImage);

	//��6����ʾЧ��ͼ
	namedWindow("2srcImage");
	imshow("2srcImage",srcImage);



	//=================����ɫͨ�����֡�=================
	//	��������ͨ�����-��ɫ��������
	//============================================

	//��0��������ر���
	Mat  imageRedChannel;

	//��1�����¶���ͼƬ
	logoImage= imread("../dota_logo.jpg",0);
	srcImage= imread("../dota_jugg.jpg");

	if( !logoImage.data ) { printf("Oh��no����ȡlogoImage����~�� \n"); return false; }
	if( !srcImage.data ) { printf("Oh��no����ȡsrcImage����~�� \n"); return false; }

	//��2����һ����ͨ��ͼ��ת����������ͨ��ͼ��
	split(srcImage,channels);//����ɫ��ͨ��

	//��3����ԭͼ�ĺ�ɫͨ�����÷��ظ�imageBlueChannel��ע�������ã��൱�����ߵȼۣ��޸�����һ����һ�����ű�
	imageRedChannel= channels.at(2);
	//��4����ԭͼ�ĺ�ɫͨ���ģ�500,250�����괦���·���һ�������logoͼ���м�Ȩ���������õ��Ļ�Ͻ���浽imageRedChannel��
	addWeighted(imageRedChannel(Rect(500,250,logoImage.cols,logoImage.rows)),1.0,
		logoImage,0.5,0.,imageRedChannel(Rect(500,250,logoImage.cols,logoImage.rows)));

	//��5�������������ĵ�ͨ�����ºϲ���һ����ͨ��
	merge(channels,srcImage);

	//��6����ʾЧ��ͼ
	namedWindow("3srcImage");
	imshow("3srcImage",srcImage);

	return true;
}


