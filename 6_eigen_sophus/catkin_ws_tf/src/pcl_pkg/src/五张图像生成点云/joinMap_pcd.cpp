#include <iostream>
#include <fstream>
using namespace std;
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Geometry> 
#include <boost/format.hpp>  // for formating strings
#include <pcl/point_types.h> 
#include <pcl/io/pcd_io.h> 
#include <pcl/visualization/pcl_visualizer.h>

int main( int argc, char** argv )
{
    vector<cv::Mat> colorImgs, depthImgs;    // 彩色图和深度图

    // 欧氏变换矩阵使用 Eigen::Isometry  虽然称为3d，实质上是4＊4的矩阵
    //Eigen::Isometry3d T=Eigen::Isometry3d::Identity();
    vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> poses;  // 相机位姿, 变换矩阵　4×4

    // 相对与可执行文件所在位置的相对路径
    ifstream fin("../pose.txt");
    if (!fin)
    {
        cerr<<"请在有pose.txt的目录下运行此程序"<<endl;
        return 1;
    }
    // 遍历五张图片, 还有位姿文件，把图片colorImgs、depthImgs和位姿存储在vector中
    for ( int i=0; i<5; i++ )
    {
        // 相对与可执行文件所在位置的相对路径,图像文件格式 批量读取图片
        boost::format fmt( "../%s/%d.%s" );
        colorImgs.push_back( cv::imread( (fmt%"color"%(i+1)%"png").str() ));
        depthImgs.push_back( cv::imread( (fmt%"depth"%(i+1)%"pgm").str(), -1 )); // 使用-1读取原始图像
        
        double data[7] = {0};
        for ( auto& d:data )
        {
            fin>>d;
            cout << d << endl;
        }

        Eigen::Quaterniond q( data[6], data[3], data[4], data[5] );
        Eigen::Isometry3d T(q);
        T.pretranslate( Eigen::Vector3d( data[0], data[1], data[2] ));
        cout << T.matrix() << endl;
        poses.push_back( T );
    }
    
    // 计算点云并拼接
    // 相机内参 
    double cx = 325.5;
    double cy = 253.5;
    double fx = 518.0;
    double fy = 519.0;
    double depthScale = 1000.0;
    
    cout<<"正在将图像转换为点云..."<<endl;
    
    // 定义点云使用的格式：这里用的是XYZRGB
    // 定义点云每一个点
    typedef pcl::PointXYZRGB PointT;
    // 定义存储每一个点的点云
    // template <typename PointT>
    // class PointCloud
    // PointCloud 是模板类，根据传入的参数初始化对应的对象
    typedef pcl::PointCloud<PointT> PointCloud;
    
    // 新建一个点云
    PointCloud::Ptr pointCloud( new PointCloud ); 
    for ( int i=0; i<5; i++ )
    {
        cout<<"转换图像中: "<<i+1<<endl; 
        cv::Mat color = colorImgs[i]; 
        cv::Mat depth = depthImgs[i];
        Eigen::Isometry3d T = poses[i];
        // 将5张图片 像素坐标转换到相机坐标 之后转换到世界坐标存储到点云格式的变量中
        // for循环之后用pcl的相关函数将点云转换到pcl能够显示的格式
        // 遍历每一个像素
        for ( int v=0; v<color.rows; v++ )
            for ( int u=0; u<color.cols; u++ )
            {
                //  根据像素的坐标，获取像素对应的深度数值，
                unsigned int d = depth.ptr<unsigned short> ( v )[u]; // 深度值
                if ( d==0 ) continue; // 为0表示没有测量到
                Eigen::Vector3d point;
                // 将图像的像素坐标通过内参矩阵K转换为相机坐标系下空间点的坐标
                point[2] = double(d)/depthScale; // dep(u,v)/s
                point[0] = (u-cx)*point[2]/fx;// x= (u - cx)z/fx
                point[1] = (v-cy)*point[2]/fy;// y= (u - cy)z/fy
                // 将图像相机坐标系通过外参矩阵T转换到世界坐标系
                // 每张图像都有自己的坐标系，所以要统一一个坐标系，
                // 把大家都转化到这个坐标系下面，这样显示才不会乱掉。
                Eigen::Vector3d pointWorld = T*point;
                //Eigen::Vector3d pointWorld = point;

                // 把世界坐标系下面的点，放到点云的每个点里面
                PointT p ;
                p.x = pointWorld[0];
                p.y = pointWorld[1];
                p.z = pointWorld[2];

                // 这里是为每一个像素填充颜色

                cout << "矩阵的维度: " << color.dims << endl;
                cout << "矩阵的行数: " << color.rows << endl;
                cout << "矩阵的列数: " << color.cols << endl;
                cout << "color.step: " << color.step << endl;
                cout << "矩阵中一行元素的字节数:　" << color.step[0] << endl;
                cout << "矩阵中一个元素的字节数(elemSize):　" << color.step[1] << endl;
                cout << "step1 = step / elemSize1:　" << color.step1(0) << endl;
                cout << "矩阵一个元素的通道数（channel()）:　" << color.step1(1) << endl;
                cout << "矩阵元素拥有的通道数: " << color.channels() << endl;
                cout << "表示了矩阵中元素的类型以及矩阵的通道个数: " << color.type() << endl;
                cout << "矩阵中元素的一个通道的数据类型: " << color.depth() << endl;
                cout << "矩阵一个元素占用的字节数: " << color.elemSize() << endl;
                cout << "矩阵元素一个通道占用的字节数: " << color.elemSize1() << endl;

                //矩阵的维度: 2
                //矩阵的行数: 480
                //矩阵的列数: 640
                //color.step: 1920
                //矩阵中一行元素的字节数:　1920
                //矩阵中一个元素的字节数(elemSize):　3
                //step1 = step / elemSize1:　1920
                //矩阵一个元素的通道数（channel()）:　3
                //矩阵元素拥有的通道数: 3
                //表示了矩阵中元素的类型以及矩阵的通道个数: 16
                //矩阵中元素的一个通道的数据类型: 0
                //矩阵一个元素占用的字节数: 3
                //矩阵元素一个通道占用的字节数: 1


                // rgb是三通道的BGR格式图，所以按下面的顺序获取颜色
                // m是行坐标，ｎ是列坐标
                // p.b = rgb.ptr<uchar>(m)[n * 3];
                // p.g = rgb.ptr<uchar>(m)[n * 3 + 1];
                // p.r = rgb.ptr<uchar>(m)[n * 3 + 2];

                // 理解的时候可以假设ｖ和ｕ等于０
                // 3 是通道数　
                // color.step = 1920 = 矩阵的列数640 * 矩阵元素的通道数3
                // 所以像素的一行就是color.step个字节，
                // 所以就是一行行的遍历，一行里面每三列代表一个像素，每一列代表一个颜色通道，
                // 每一列就是一个通道的颜色表示，是8位１个字节。
                p.b = color.data[ v*color.step+u*3+0 ];
                p.g = color.data[ v*color.step+u*3+1 ];
                p.r = color.data[ v*color.step+u*3+2 ];
                pointCloud->points.push_back( p );
            }
    }
    
    pointCloud->is_dense = false;
    cout<<"点云共有"<<pointCloud->size()<<"个点."<<endl;
    pcl::io::savePCDFileBinary("../map.pcd", *pointCloud );
    return 0;
}
