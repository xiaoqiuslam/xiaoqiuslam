#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace cv;
using namespace std;



#define NTESTS 14
#define NITERATIONS 20



void colorReduce0(Mat &image, int div=64) {

	  int nl= image.rows; //����
	  int nc= image.cols * image.channels(); //ÿ��Ԫ�ص���Ԫ������
              
      for (int j=0; j<nl; j++) 
	  {

		  uchar* data= image.ptr<uchar>(j);

          for (int i=0; i<nc; i++) 
		  {
 
            //-------------��ʼ����ÿ������-------------------
                 
                  data[i]= data[i]/div*div + div/2;
 
            //-------------�������ش���------------------------
 
            } //���д�������                  
      }
}

//-----------------------------------����������-------------------------------------------------
//		˵�������� .ptr �� * ++ 
//-------------------------------------------------------------------------------------------------
void colorReduce1(Mat &image, int div=64) {

	  int nl= image.rows; //����
	  int nc= image.cols * image.channels(); //ÿ��Ԫ�ص���Ԫ������
              
      for (int j=0; j<nl; j++) 
	  {

		  uchar* data= image.ptr<uchar>(j);

          for (int i=0; i<nc; i++) 
		  {
 
            //-------------��ʼ����ÿ������-------------------
                 
				 *data++= *data/div*div + div/2;
 
            //-------------�������ش���------------------------
 
            } //���д�������              
      }
}

//-----------------------------------------����������-------------------------------------------
//		˵��������.ptr �� * ++ �Լ�ģ����
//-------------------------------------------------------------------------------------------------
void colorReduce2(Mat &image, int div=64) {

	  int nl= image.rows; //����
	  int nc= image.cols * image.channels(); //ÿ��Ԫ�ص���Ԫ������
              
      for (int j=0; j<nl; j++) 
	  {

		  uchar* data= image.ptr<uchar>(j);

          for (int i=0; i<nc; i++) 
		  {
 
            //-------------��ʼ����ÿ������-------------------
       
			      int v= *data;
                  *data++= v - v%div + div/2;
 
            //-------------�������ش���------------------------
 
            } //���д�������                   
      }
}

//----------------------------------------�������ġ�---------------------------------------------
//		˵��������.ptr �� * ++ �Լ�λ����
//----------------------------------------------------------------------------------------------------
void colorReduce3(Mat &image, int div=64) {

	  int nl= image.rows; //����
	  int nc= image.cols * image.channels(); //ÿ��Ԫ�ص���Ԫ������
	  int n= static_cast<int>(log(static_cast<double>(div))/log(2.0));
	  //����ֵ
	  uchar mask= 0xFF<<n; // e.g. ���� div=16, mask= 0xF0
              
      for (int j=0; j<nl; j++) {

		  uchar* data= image.ptr<uchar>(j);

          for (int i=0; i<nc; i++) {
 
            //------------��ʼ����ÿ������-------------------
                 
            *data++= *data&mask + div/2;
 
            //-------------�������ش���------------------------
            }  //���д�������            
      }
}


//----------------------------------------�������塿----------------------------------------------
//		˵��������ָ����������
//---------------------------------------------------------------------------------------------------
void colorReduce4(Mat &image, int div=64) {

	  int nl= image.rows; //����
	  int nc= image.cols * image.channels(); //ÿ��Ԫ�ص���Ԫ������
	  int n= static_cast<int>(log(static_cast<double>(div))/log(2.0));
	  int step= image.step; //��Ч����
	  //����ֵ
	  uchar mask= 0xFF<<n; // e.g. ���� div=16, mask= 0xF0
              
      //��ȡָ��ͼ�񻺳�����ָ��
	  uchar *data= image.data;

      for (int j=0; j<nl; j++)
	  {

          for (int i=0; i<nc; i++) 
		  {
 
            //-------------��ʼ����ÿ������-------------------
                 
            *(data+i)= *data&mask + div/2;
 
            //-------------�������ش���------------------------
 
            } //���д�������              

            data+= step;  // next line
      }
}

//---------------------------------------����������----------------------------------------------
//		˵�������� .ptr �� * ++�Լ�λ���㡢image.cols * image.channels()
//-------------------------------------------------------------------------------------------------
void colorReduce5(Mat &image, int div=64) {

	  int nl= image.rows; //����
	  int n= static_cast<int>(log(static_cast<double>(div))/log(2.0));
	  //����ֵ
	  uchar mask= 0xFF<<n; // e.g. ����div=16, mask= 0xF0
              
      for (int j=0; j<nl; j++) 
	  {

		  uchar* data= image.ptr<uchar>(j);

          for (int i=0; i<image.cols * image.channels(); i++) 
		  {
 
            //-------------��ʼ����ÿ������-------------------
                 
            *data++= *data&mask + div/2;
 
            //-------------�������ش���------------------------
 
            } //���д�������            
      }
}

// -------------------------------------�������ߡ�----------------------------------------------
//		˵��������.ptr �� * ++ �Լ�λ����(continuous)
//-------------------------------------------------------------------------------------------------
void colorReduce6(Mat &image, int div=64) {

	  int nl= image.rows; //����
	  int nc= image.cols * image.channels(); //ÿ��Ԫ�ص���Ԫ������

	  if (image.isContinuous())  
	  {
		  //���������
		  nc= nc*nl; 
		  nl= 1;  // Ϊһά����
	   }

	  int n= static_cast<int>(log(static_cast<double>(div))/log(2.0));
	  //����ֵ
	  uchar mask= 0xFF<<n; // e.g. ����div=16, mask= 0xF0
              
      for (int j=0; j<nl; j++) {

		  uchar* data= image.ptr<uchar>(j);

          for (int i=0; i<nc; i++) {
 
            //-------------��ʼ����ÿ������-------------------
                 
            *data++= *data&mask + div/2;
 
            //-------------�������ش���------------------------
 
            } //���д�������                   
      }
}

//------------------------------------�������ˡ�------------------------------------------------
//		˵�������� .ptr �� * ++ �Լ�λ���� (continuous+channels)
//-------------------------------------------------------------------------------------------------
void colorReduce7(Mat &image, int div=64) {

	  int nl= image.rows; //����
	  int nc= image.cols ; //����

	  if (image.isContinuous())  
	  {
		  //���������
		  nc= nc*nl; 
		  nl= 1;  // Ϊһά����
	   }

	  int n= static_cast<int>(log(static_cast<double>(div))/log(2.0));
	  //����ֵ
	  uchar mask= 0xFF<<n; // e.g. ����div=16, mask= 0xF0
              
      for (int j=0; j<nl; j++) {

		  uchar* data= image.ptr<uchar>(j);

          for (int i=0; i<nc; i++) {
 
            //-------------��ʼ����ÿ������-------------------
                 
            *data++= *data&mask + div/2;
            *data++= *data&mask + div/2;
            *data++= *data&mask + div/2;
 
            //-------------�������ش���------------------------
 
            } //���д�������                    
      }
}


// -----------------------------------�������š� ------------------------------------------------
//		˵��������Mat_ iterator
//-------------------------------------------------------------------------------------------------
void colorReduce8(Mat &image, int div=64) {

	  //��ȡ������
	  Mat_<Vec3b>::iterator it= image.begin<Vec3b>();
	  Mat_<Vec3b>::iterator itend= image.end<Vec3b>();

	  for ( ; it!= itend; ++it) {
        
		//-------------��ʼ����ÿ������-------------------

        (*it)[0]= (*it)[0]/div*div + div/2;
        (*it)[1]= (*it)[1]/div*div + div/2;
        (*it)[2]= (*it)[2]/div*div + div/2;

        //-------------�������ش���------------------------
	  }//���д�������  
}

//-------------------------------------������ʮ��-----------------------------------------------
//		˵��������Mat_ iterator�Լ�λ����
//-------------------------------------------------------------------------------------------------
void colorReduce9(Mat &image, int div=64) {

	  // div������2����
	  int n= static_cast<int>(log(static_cast<double>(div))/log(2.0));
	  //����ֵ
	  uchar mask= 0xFF<<n; // e.g. ���� div=16, mask= 0xF0

	  // ��ȡ������
	  Mat_<Vec3b>::iterator it= image.begin<Vec3b>();
	  Mat_<Vec3b>::iterator itend= image.end<Vec3b>();

	  //ɨ������Ԫ��
	  for ( ; it!= itend; ++it) 
	  {
        
		//-------------��ʼ����ÿ������-------------------

        (*it)[0]= (*it)[0]&mask + div/2;
        (*it)[1]= (*it)[1]&mask + div/2;
        (*it)[2]= (*it)[2]&mask + div/2;

        //-------------�������ش���------------------------
	  }//���д�������  
}

//------------------------------------������ʮһ��---------------------------------------------
//		˵��������Mat Iterator_
//-------------------------------------------------------------------------------------------------
void colorReduce10(Mat &image, int div=64) {

	  //��ȡ������
	  Mat_<Vec3b> cimage= image;
	  Mat_<Vec3b>::iterator it=cimage.begin();
	  Mat_<Vec3b>::iterator itend=cimage.end();

	  for ( ; it!= itend; it++) { 
        
		//-------------��ʼ����ÿ������-------------------

        (*it)[0]= (*it)[0]/div*div + div/2;
        (*it)[1]= (*it)[1]/div*div + div/2;
        (*it)[2]= (*it)[2]/div*div + div/2;

        //-------------�������ش���------------------------
	  }
}

//--------------------------------------������ʮ����--------------------------------------------
//		˵�������ö�̬��ַ�������at
//-------------------------------------------------------------------------------------------------
void colorReduce11(Mat &image, int div=64) {

	  int nl= image.rows; //����
	  int nc= image.cols; //����
              
      for (int j=0; j<nl; j++) 
	  {
          for (int i=0; i<nc; i++) 
		  {
 
            //-------------��ʼ����ÿ������-------------------
                 
                  image.at<Vec3b>(j,i)[0]=	 image.at<Vec3b>(j,i)[0]/div*div + div/2;
                  image.at<Vec3b>(j,i)[1]=	 image.at<Vec3b>(j,i)[1]/div*div + div/2;
                  image.at<Vec3b>(j,i)[2]=	 image.at<Vec3b>(j,i)[2]/div*div + div/2;
 
            //-------------�������ش���------------------------
 
            } //���д�������                 
      }
}

//----------------------------------������ʮ����----------------------------------------------- 
//		˵��������ͼ������������
//-------------------------------------------------------------------------------------------------
void colorReduce12(const Mat &image, //����ͼ��
                 Mat &result,      // ���ͼ��
                 int div=64) {

	  int nl= image.rows; //����
	  int nc= image.cols ; //����

	  //׼���ó�ʼ�����Mat�����ͼ��
	  result.create(image.rows,image.cols,image.type());

	  //��������������ͼ��
	  nc= nc*nl; 
	  nl= 1;  //��ά����

	  int n= static_cast<int>(log(static_cast<double>(div))/log(2.0));
	  //����ֵ
	  uchar mask= 0xFF<<n; // e.g.����div=16, mask= 0xF0
              
      for (int j=0; j<nl; j++) {

		  uchar* data= result.ptr<uchar>(j);
		  const uchar* idata= image.ptr<uchar>(j);

          for (int i=0; i<nc; i++) {
 
            //-------------��ʼ����ÿ������-------------------
                 
            *data++= (*idata++)&mask + div/2;
            *data++= (*idata++)&mask + div/2;
            *data++= (*idata++)&mask + div/2;
 
            //-------------�������ش���------------------------
 
          } //���д�������                   
      }
}

//--------------------------------------������ʮ�ġ�------------------------------------------- 
//		˵�������ò���������
//-------------------------------------------------------------------------------------------------
void colorReduce13(Mat &image, int div=64) {
	
	  int n= static_cast<int>(log(static_cast<double>(div))/log(2.0));
	  //����ֵ
	  uchar mask= 0xFF<<n; // e.g. ����div=16, mask= 0xF0

	  //����ɫ�ʻ�ԭ
	  image=(image&Scalar(mask,mask,mask))+Scalar(div/2,div/2,div/2);
}




//-----------------------------------��ShowHelpText( )������-----------------------------
//		���������һЩ������Ϣ
//----------------------------------------------------------------------------------------------
void ShowHelpText()
{
	//�����ӭ��Ϣ��OpenCV�汾
	printf("\n\n\t\t\t�ǳ���л����OpenCV3������š�һ�飡\n");
	printf("\n\n\t\t\t��Ϊ����OpenCV3��ĵ�24������ʾ������\n");
	printf("\n\n\t\t\t   ��ǰʹ�õ�OpenCV�汾Ϊ��" CV_VERSION );
	printf("\n\n  ----------------------------------------------------------------------------\n");

	printf("\n\n���ڽ��д�ȡ���������Եȡ���\n\n");
}




//-----------------------------------��main( )������--------------------------------------------
//		����������̨Ӧ�ó������ں��������ǵĳ�������￪ʼ
//-------------------------------------------------------------------------------------------------
int main( )
{
	int64 t[NTESTS],tinit;
	Mat image0;
	Mat image1;
	Mat image2;

	system("color 4F");

	ShowHelpText();

	image0= imread("../1_read.jpg");
	if (!image0.data)
		return 0; 

	//ʱ��ֵ��Ϊ0
	for (int i=0; i<NTESTS; i++)
		t[i]= 0;


	// ����ظ�����
	int n=NITERATIONS;
	for (int k=0; k<n; k++)
	{
		cout << k << " of " << n << endl; 

		image1= imread("../1_read.jpg");
		//������һ������.ptr �� []
	    tinit= getTickCount();
		colorReduce0(image1);
		t[0]+= getTickCount()-tinit;

		//�������������� .ptr �� * ++ 
		image1= imread("../1_read.jpg");
	    tinit= getTickCount();
		colorReduce1(image1);
		t[1]+= getTickCount()-tinit;

		//��������������.ptr �� * ++ �Լ�ģ����
		image1= imread("../1_read.jpg");
	    tinit= getTickCount();
		colorReduce2(image1);
		t[2]+= getTickCount()-tinit;

		//�������ġ� ����.ptr �� * ++ �Լ�λ����
		image1= imread("../1_read.jpg");
	    tinit= getTickCount();
		colorReduce3(image1);
		t[3]+= getTickCount()-tinit;

		//�������塿 ����ָ�����������
		image1= imread("../1_read.jpg");
	    tinit= getTickCount();
		colorReduce4(image1);
		t[4]+= getTickCount()-tinit;

		//�������������� .ptr �� * ++�Լ�λ���㡢image.cols * image.channels()
		image1= imread("../1_read.jpg");
	    tinit= getTickCount();
		colorReduce5(image1);
		t[5]+= getTickCount()-tinit;

		//�������ߡ�����.ptr �� * ++ �Լ�λ����(continuous)
		image1= imread("../1_read.jpg");
	    tinit= getTickCount();
		colorReduce6(image1);
		t[6]+= getTickCount()-tinit;

		//�������ˡ����� .ptr �� * ++ �Լ�λ���� (continuous+channels)
		image1= imread("../1_read.jpg");
	    tinit= getTickCount();
		colorReduce7(image1);
		t[7]+= getTickCount()-tinit;

		//�������š� ����Mat_ iterator
		image1= imread("../1_read.jpg");
	    tinit= getTickCount();
		colorReduce8(image1);
		t[8]+= getTickCount()-tinit;

		//������ʮ�� ����Mat_ iterator�Լ�λ����
		image1= imread("../1_read.jpg");
	    tinit= getTickCount();
		colorReduce9(image1);
		t[9]+= getTickCount()-tinit;

		//������ʮһ������Mat Iterator_
		image1= imread("../1_read.jpg");
	    tinit= getTickCount();
		colorReduce10(image1);
		t[10]+= getTickCount()-tinit;

		//������ʮ���� ���ö�̬��ַ�������at
		image1= imread("../1_read.jpg");
	    tinit= getTickCount();
		colorReduce11(image1);
		t[11]+= getTickCount()-tinit;

		//������ʮ���� ����ͼ������������
		image1= imread("../1_read.jpg");
	    tinit= getTickCount();
		Mat result;
		colorReduce12(image1, result);
		t[12]+= getTickCount()-tinit;
		image2= result;
		
		//������ʮ�ġ� ���ò���������
		image1= imread("../1_read.jpg");
	    tinit= getTickCount();
		colorReduce13(image1);
		t[13]+= getTickCount()-tinit;

		//------------------------------
	}
	 //���ͼ��   
	imshow("image0",image0);
	waitKey();
	imshow("image2",image2);
	waitKey();
	imshow("image1",image1);
	waitKey();

	// ���ƽ��ִ��ʱ��
	cout << endl << "-------------------------------------------" << endl << endl;
	cout << "\n������һ������.ptr �� []�ķ�������ʱ��Ϊ " << 1000.*t[0]/getTickFrequency()/n << "ms" << endl;
	cout << "\n�������������� .ptr �� * ++ �ķ�������ʱ��Ϊ" << 1000.*t[1]/getTickFrequency()/n << "ms" << endl;
	cout << "\n��������������.ptr �� * ++ �Լ�ģ�����ķ�������ʱ��Ϊ" << 1000.*t[2]/getTickFrequency()/n << "ms" << endl;
	cout << "\n�������ġ�����.ptr �� * ++ �Լ�λ�����ķ�������ʱ��Ϊ" << 1000.*t[3]/getTickFrequency()/n << "ms" << endl;
	cout << "\n�������塿����ָ����������ķ�������ʱ��Ϊ" << 1000.*t[4]/getTickFrequency()/n << "ms" << endl;
	cout << "\n�������������� .ptr �� * ++�Լ�λ���㡢channels()�ķ�������ʱ��Ϊ" << 1000.*t[5]/getTickFrequency()/n << "ms" << endl;
	cout << "\n�������ߡ�����.ptr �� * ++ �Լ�λ����(continuous)�ķ�������ʱ��Ϊ" << 1000.*t[6]/getTickFrequency()/n << "ms" << endl;
	cout << "\n�������ˡ����� .ptr �� * ++ �Լ�λ���� (continuous+channels)�ķ�������ʱ��Ϊ" << 1000.*t[7]/getTickFrequency()/n << "ms" << endl;
	cout << "\n�������š�����Mat_ iterator �ķ�������ʱ��Ϊ" << 1000.*t[8]/getTickFrequency()/n << "ms" << endl;
	cout << "\n������ʮ������Mat_ iterator�Լ�λ����ķ�������ʱ��Ϊ" << 1000.*t[9]/getTickFrequency()/n << "ms" << endl;
	cout << "\n������ʮһ������Mat Iterator_�ķ�������ʱ��Ϊ" << 1000.*t[10]/getTickFrequency()/n << "ms" << endl;	
	cout << "\n������ʮ�������ö�̬��ַ�������at �ķ�������ʱ��Ϊ" << 1000.*t[11]/getTickFrequency()/n << "ms" << endl;	
	cout << "\n������ʮ��������ͼ�������������ķ�������ʱ��Ϊ" << 1000.*t[12]/getTickFrequency()/n << "ms" << endl;	
	cout << "\n������ʮ�ġ����ò��������صķ�������ʱ��Ϊ" << 1000.*t[13]/getTickFrequency()/n << "ms" << endl;	
	
	return 0;
}
