#include <iostream>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Geometry> 
#include <boost/format.hpp>
#include <pcl/point_types.h> 
#include <pcl/io/pcd_io.h> 
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;

int main( int argc, char** argv ){
    // 彩色图和深度图
    vector<cv::Mat> colorImgs, depthImgs;
    // 相机位姿 变换矩阵　4×4
    vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> poses;         
    
    // 相对与可执行文件所在位置的相对路径
    ifstream fin("../pose.txt");
    if (!fin)
    {
        cerr<<"请在有pose.txt的目录下运行此程序"<<endl;
        return 1;
    }
    
    for ( int i=0; i<5; i++ )
    {
        // 相对与可执行文件所在位置的相对路径,图像文件格式 批量读取图片
        boost::format fmt( "../%s/%d.%s" ); 
        colorImgs.push_back( cv::imread( (fmt%"color"%(i+1)%"png").str() ));
        // 使用-1读取原始图像
        depthImgs.push_back( cv::imread( (fmt%"depth"%(i+1)%"pgm").str(), -1 )); 
        
        double data[7] = {0};
        for ( auto& d:data )
            fin>>d;
        Eigen::Quaterniond q( data[6], data[3], data[4], data[5] );
        Eigen::Isometry3d T(q);
        T.pretranslate( Eigen::Vector3d( data[0], data[1], data[2] ));
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
    
    // 定义点云使用的格式：这里用的是XYZRGB 定义点云每一个点
    typedef pcl::PointXYZRGB PointT; 
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
                //  根据像素的坐标，获取像素对应的深度数值
                unsigned int d = depth.ptr<unsigned short> ( v )[u]; 
                if ( d==0 ) continue; // 为0表示没有测量到
                Eigen::Vector3d point; 
                // 将图像的像素坐标通过内参矩阵K转换为相机坐标系下空间点的坐标
                point[2] = double(d)/depthScale; 
                // x= (u - cx)z/fx
                // y= (u - cy)z/fy
                point[0] = (u-cx)*point[2]/fx;
                point[1] = (v-cy)*point[2]/fy; 
                // 将图像相机坐标系通过外参矩阵T转换到世界坐标系
                // 每张图像都有自己的坐标系，所以要统一一个坐标系，
                // 把大家都转化到这个坐标系下面，这样显示才不会乱
                Eigen::Vector3d pointWorld = T*point;
                
                // 把世界坐标系下面的点，放到点云的每个点里面
                PointT p ;
                p.x = pointWorld[0];
                p.y = pointWorld[1];
                p.z = pointWorld[2];
                p.b = color.data[ v*color.step+u*color.channels() ];
                p.g = color.data[ v*color.step+u*color.channels()+1 ];
                p.r = color.data[ v*color.step+u*color.channels()+2 ];
                pointCloud->points.push_back( p );
            }
    }
    
    pointCloud->is_dense = false;
    cout<<"点云共有"<<pointCloud->size()<<"个点."<<endl;
    pcl::io::savePCDFileBinary("../map.pcd", *pointCloud );
    return 0;
}
