## cv::imread

image = cv::imread("../x.png", -1);

1 = IMREAD_COLOR 默认参数 将图像转换成3通道BGR彩色图像加载彩色图片
0 = IMREAD_GRAYSCALE 将图像转成单通道灰度图像后读取
-1 = IMREAD_UNCHANGED 按照图像原样读取 保留第4通道Alpha通道

## cv::namedWindow

cv::namedWindow( "name", cv::WINDOW_AUTOSIZE);

WINDOW_NORMAL 显示图像后，允许用户随意调整图像窗口大小
WINDOW_AUTOSIZE 根据图像大小显示窗口，不允许用户调整图像大小

## cv::imshow & cv::waitKey

imshow( "winname", mat);

winname 要显示图像的窗口的名字，用字符串形式赋值
mat 要显示的图像矩阵

cv::waitKey(0);

等待任意按键按下退出,不加这一句窗口会一闪而过,等待6000 ms后窗口自动关闭 waitKey(6000);

## cv::Mat rows & cols

矩阵行数/高为image.rows
矩阵列数/宽为image.cols

## cv::Mat type


type表示矩阵中元素的类型以及矩阵的通道个数 是一系列的预定义的常量 命名规则为CV_(位数)+(数据类型)+(通道数)

U表示Unsigned无符号整数类型, 即其内部元素的值不可以为负数
S表示Signed有符号整数类型, 其值存在负数
F则表示浮点数类型,即矩阵的内部元素值可以为小数(32对应单精度float类型, 64对应双精度double类型)

CV_8U -> uchar
CV_8S -> char
CV_16S -> short
CV_16U -> ushort
CV_32S -> int
CV_32F -> float
CV_64F -> double
CV_8UC3 = 16
CV_16UC3 = 18

C1-C4表示对应的通道数,即有1-4个通道

CV_8UC1---可以创建---8位无符号的单通道---灰度图片---grayImg
#define CV_8UC1 CV_MAKETYPE(CV_8U,1)  type 预定义的常量 = 0

CV_8UC3---可以创建---8位无符号的三通道---RGB彩色图像---colorImg
#define CV_8UC3 CV_MAKETYPE(CV_8U,3)  type 预定义的常量 = 16

CV_8UC4---可以创建---8位无符号的四通道---带透明色Alpha通道的RGB图像
#define CV_8UC4 CV_MAKETYPE(CV_8U,4)  type 预定义的常量 = 24



## cv::Mat depth()

Mat.depth()得到的是一个0-6的数字,分别代表不同的位数, 0和1都代表8位, 2和3都代表16位, 4和5代表32位, 6代表64位
enum{CV_8U=0, CV_8S=1, CV_16U=2, CV_16S=3, CV_32S=4, CV_32F=5, CV_64F=6}

## cv::Mat elemSize() & elemSize1()

矩阵中像素的个数为image.rows*image.cols
elemSize以8位(一个字节)为单位表示矩阵中每一个像素的字节数
以字节为单位的有效长度的step为eleSize()*cols=48
CV_8UC1, elemSize==1字节
CV_8UC3或CV_8SC3, elemSize==3字节
CV_16UC3或CV_16SC3, elemSize==6字节

单个通道下每个元素的字节数
elemSize加上一个"1"构成了elemSize1这个属性,可认为是元素内1个通道的意思,
表示Mat矩阵中每一个元素单个通道的数据大小, 以字节为单位, eleSize1==elemSize/channels

## cv::Mat step & step1()

以字节为单位, step为Mat矩阵中一行像素占用多少个字节, step=elemSize()*cols
step1()以字节为基本单位, Mat矩阵中每一个像素的大小, 累计了所有通道的elemSize1之后的值, 所以有step1==step/elemSize1


## cv::Scalar

cv::Scalar(0,0,255)每个像素由三个元素组成即三通道,初始化颜色值为(0,0,255)

## CV_8UC4

typedef Vec <uchar, 3> Vec3b
vec3b 表示每个Vec3b对象可以存储3个char(字符型)数据, 比如可以用这样的对象,去存储RGB图像中的
vec4b 表示每个Vec4b对象可以存储4个字符型数据, 可以用这样的类对象去存储4通道RGB+Alpha的图

mat.at<cv::Vec4b>(i, j);
从mat中取出一个像素,像素的类型是Vec4b该类型含义是, 有4个uchar类型的元素
其中rgba[0], rgba[1], rgba[2]代表像素三原色BGR, 即为蓝色Blue, Green绿色, 红色Red, rgba[3]代表像素的Alpha值表示像素透明度



## .at 遍历单通道图像像素

行优先
y=ROWS=height 
x=COLS=width

通过at方法读取元素需要在后面跟上<数据类型>以坐标的形式给出需要读取的元素坐标(行数,列数)
.at<uchar>(y, x)
.at<double>(y, x)




## .ptr 遍历单通道图像像素

for(int i = 0; i < image_gray.rows; i++){
    uchar* data = image_gray.ptr<uchar>(i);
    for(int j = 0; j < image_gray.cols * image_gray.channels(); j++){
        std::cout << float(data[j]) << " ";
    }
}


float grayscale = float (image_gray.ptr<uchar> ( cvRound ( kp.pt.y ) ) [ cvRound ( kp.pt.x ) ] );





## .at 遍历三通道图像像素

OpenCV中定义cv::Vec3b, cv::Vec3s, cv::Vec3w, cv::Vec3d, cv::Vec3f, cv::Vec3i 六种类型表示同一个元素的三个通道数据
数字表示通道的个数, 最后一位是数据类型的缩写, b是uchar类型的缩写, s是short类型的缩写, w是ushort类型的缩写, d是double类型的缩写, f是float类型的缩写, i是int类型的缩写

Vec3b描述RGB颜色
cv::Vec3b color;
color[0]=0;//B分量
color[1]=0;//G分量
color[2]=255;//R分量


for(int y = 0; y < mat.rows; y++){
    for(int x = 0; x < mat.cols; x++){
        std::cout << "mat.at<cv::Vec3b>(y,x)[0]: " << int(mat.at<cv::Vec3b>(y,x)[0]) << std::endl;  //获得蓝色通道b的像素值
        std::cout << "mat.at<cv::Vec3b>(y,x)[0]: " << int(mat.at<cv::Vec3b>(y,x)[1]) << std::endl;  //获得绿色通道g的像素值
        std::cout << "mat.at<cv::Vec3b>(y,x)[0]: " << int(mat.at<cv::Vec3b>(y,x)[2]) << std::endl;  //获得红色通道r的像素值
       
    }
}


for(int y = 0; y < image_rgb.rows; y++){
    uchar* data = image_rgb.ptr<uchar>(y);
    for(int x = 0; x < image_rgb.cols * image_rgb.channels(); x++){
        std::cout << float(data[x]) << " " ;
    }
    std::cout << std::endl;
}





## .ptr 遍历单通道图像像素

for (int y = 0; y < mat.rows; y++){
    uchar* row_ptr = mat.ptr(y);// 定义一个uchar类型的row_ptr指向图像行的头指针
    for (int x = 0; x < mat.cols*mat.channels(); x++){
        // 遍历图像每一行所有通道的数据 Mat类矩阵矩阵中每一行中的每个元素都是挨着存放, 每一行中存储的数据数量为列数与通道数的乘积 即代码中指针向后移动cols*channels()-1位
        std::cout << "row_ptr[" << x << "]: " << (int)row_ptr[x] << std::endl;
    }
}

