//
// Created by q on 2019/11/18.
//
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;



int main ()
{

    /*************************************
     *  1--gray灰度图片-----------单通道图像
     *  2--RGB彩色图像------------3通道图像
     *  3--带Alph通道的RGB图像-----4通道图像
     *  Alpha通道使用8位二进制数，可以表示256级灰度，取值范围是0~255。
     *  值为255的Alpha像素用以定义不透明的彩色像素，
     *  而值为0的Alpha通道像素用以定义透明像素，
     *  介于黑白之间的灰度（值为30-255）的Alpha像素用以定义不同程度的半透明像素。
     *  用Mat (3, 4, CV_32FC3)定义一个矩阵，这时通道数channels()为3；列数cols为4；行数rows为3；矩阵中元素的个数为3*4，结果为12；
     *  每个元素的字节数为32/8*channels()，最后结果为12；以字节为单位的有效长度step为eleSize()*cols，结果为48。
     *  
     * CV_(位数）+（数据类型）+（通道数）
     *  其中CV_后面紧接的数字表示位数，分别对应8bit（0~255或者-128~127），16bit（0~65535或者-32768~32767），
     *  U表示Unsigned无符号整数类型，即其内部元素的值不可以为负数，
     *  S表示Signed有符号整数类型，其值存在负数，
     *  F则表示浮点数类型，即矩阵的内部元素值可以为小数（32对应单精度float类型，64对应双精度double类型）；
     *  C1~C4表示对应的通道数，即有1~4个通道
     * 【1】CV_8UC1---则可以创建----8位无符号的单通道---灰度图片------grayImg
     *  #define CV_8UC1 CV_MAKETYPE(CV_8U,1)
     *  #define CV_8UC2 CV_MAKETYPE(CV_8U,2)
     * 【2】CV_8UC3---则可以创建----8位无符号的三通道---RGB彩色图像---colorImg
     *  #define CV_8UC3 CV_MAKETYPE(CV_8U,3)
     * 【3】CV_8UC4--则可以创建-----8位无符号的四通道---带透明色的RGB图像
     *  #define CV_8UC4 CV_MAKETYPE(CV_8U,4)
     ***************************************************************************/
    // CV_8UC3 表示使用8位的 unsigned char 型，每个像素由三个元素组成三通道,初始化为（50,100,255）
    Mat M(800,800, CV_8UC3, Scalar(50,100,255));
    int step3 = (int)M.step1();
    cout << "(int)image.step1()= " << step3 << endl;
    int step4 = (int)M.step;
    cout << "(int)image.step()= " << step4 << endl;
    cv::namedWindow("M");
    cv::imshow("M", M);
    cv::waitKey(0);

    cv::Mat image_rgb(9, 9, CV_8UC3, cv::Scalar(99, 0, 0));
    for(int i = 0; i < image_rgb.rows; i++){
        uchar* data = image_rgb.ptr<uchar>(i);
        for(int j = 0; j < image_rgb.cols * image_rgb.channels(); j++){
            cout << float(data[j]) << " " ;
        }
        cout << endl;
    }


    cv::namedWindow("image_rgb", CV_WINDOW_NORMAL);
    imshow("image_rgb",image_rgb);
    waitKey(0);


    Mat r = Mat(10, 3, CV_8UC3);
    randu(r, Scalar::all(0), Scalar::all(255));
    cv::namedWindow("r", CV_WINDOW_NORMAL);
    imshow("r", r);
    cv::waitKey(0);

    //三维
    int sz[3] = {3,3,3};
    cv::Mat L(3,sz, CV_8UC(1), cv::Scalar::all(0));
    // 超过两维的矩阵：指定维数，然后传递一个指向一个数组的指针，这个数组包含每个维度的尺寸；其余的相同
    std::cout << "L = " << std::endl << " " << M << std::endl << std::endl; //格式化输出

    

    return 0;
}
