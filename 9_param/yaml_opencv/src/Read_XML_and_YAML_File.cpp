#include "opencv2/opencv.hpp"
#include <time.h>  
using namespace cv;  
using namespace std;  



int main( )  
{  
	FileStorage fs2("30_Read_XML_and_YAML_File.yaml", FileStorage::READ);

	// ��һ�ַ�������FileNode����
	int frameCount = (int)fs2["frameCount"];  

	std::string date;  
	// �ڶ��ַ�����ʹ��FileNode�����> > 
	fs2["calibrationDate"] >> date;  

	Mat cameraMatrix2, distCoeffs2;  
	fs2["cameraMatrix"] >> cameraMatrix2;  
	fs2["distCoeffs"] >> distCoeffs2;  

	cout << "frameCount: " << frameCount << endl  
		<< "calibration date: " << date << endl  
		<< "camera matrix: " << cameraMatrix2 << endl  
		<< "distortion coeffs: " << distCoeffs2 << endl;  

	FileNode features = fs2["features"];  
	FileNodeIterator it = features.begin(), it_end = features.end();  
	int idx = 0;  
	std::vector<uchar> lbpval;  

	//ʹ��FileNodeIterator��������
	for( ; it != it_end; ++it, idx++ )  
	{  
		cout << "feature #" << idx << ": ";  
		cout << "x=" << (int)(*it)["x"] << ", y=" << (int)(*it)["y"] << ", lbp: (";  
		// ����Ҳ����ʹ��ʹ��filenode > > std::vector�����������׵Ķ���ֵ����
		(*it)["lbp"] >> lbpval;  
		for( int i = 0; i < (int)lbpval.size(); i++ )  
			cout << " " << (int)lbpval[i];  
		cout << ")" << endl;  
	}  
	fs2.release();  

	printf("\n");
	getchar();

	return 0;  
}  
