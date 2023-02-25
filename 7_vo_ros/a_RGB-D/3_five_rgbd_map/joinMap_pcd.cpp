#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Geometry>
#include <boost/format.hpp>
#include <pcl/point_types.h> 
#include <pcl/io/pcd_io.h> 
#include <pcl/visualization/pcl_visualizer.h>

int main( int argc, char** argv )
{
    ifstream fin("../pose.txt");
    std::vector<cv::Mat> colorImgs, depthImgs;
    // 4×4变换矩阵　
    std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> poses;
    for ( int i=0; i<5; i++ )
    {
        boost::format fmt( "../%s/%d.%s" );
        colorImgs.push_back( cv::imread( (fmt%"color"%(i+1)%"png").str() ));
        depthImgs.push_back( cv::imread( (fmt%"depth"%(i+1)%"pgm").str(), -1 ));
        
        double data[7] = {0};
        for ( auto& d:data )
        {
            fin>>d;
        }
        // -0.228993 0.00645704 0.0287837 -0.0004327 -0.113131 -0.0326832 0.993042
        Eigen::Quaterniond quaterniond( data[6], data[3], data[4], data[5] );
        Eigen::Isometry3d transformation_matrix(quaterniond);
        transformation_matrix.pretranslate( Eigen::Vector3d( data[0], data[1], data[2] ));
        std::cout << transformation_matrix.matrix() << std::endl;
        poses.push_back(transformation_matrix);
    }

    double cx = 325.5;
    double cy = 253.5;
    double fx = 518.0;
    double fy = 519.0;
    double depthScale = 1000.0;
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud( new pcl::PointCloud<pcl::PointXYZRGB> );
    for ( int i=0; i<5; i++ )
    {
        cv::Mat color = colorImgs[i];
        cv::Mat depth = depthImgs[i];
        Eigen::Isometry3d transformation_matrix_iter = poses[i];
        // 将5张图片 像素坐标转换到相机坐标 之后转换到世界坐标存储到点云格式的变量中
        for ( int v=0; v<color.rows; v++ )
            for ( int u=0; u<color.cols; u++ )
            {
                unsigned int d = depth.ptr<unsigned short> ( v )[u];
                if ( d==0 )
                    continue;
                Eigen::Vector3d point;
                point[2] = double(d)/depthScale; // dep(u,v)/s
                point[0] = (u-cx)*point[2]/fx;// x= (u - cx)z/fx
                point[1] = (v-cy)*point[2]/fy;// y= (u - cy)z/fy
                Eigen::Vector3d eigen_point_world = transformation_matrix_iter*point;
                pcl::PointXYZRGB pcl_point_world;
                pcl_point_world.x = eigen_point_world[0];
                pcl_point_world.y = eigen_point_world[1];
                pcl_point_world.z = eigen_point_world[2];

                // color.step = 1920 = 矩阵的列数640 * 矩阵元素的通道数3
                // 所以像素的一行就是color.step个字节，
                // 所以就是一行行的遍历，一行里面每三列代表一个像素，每一列代表一个颜色通道，
                // 每一列就是一个通道的颜色表示，是8位１个字节。
                pcl_point_world.b = color.data[ v*color.step+u*3+0 ];
                pcl_point_world.g = color.data[ v*color.step+u*3+1 ];
                pcl_point_world.r = color.data[ v*color.step+u*3+2 ];

                //p.b = rgb.ptr<uchar>(u)[v*3];
                //p.g = rgb.ptr<uchar>(u)[v*3+1];
                //p.r = rgb.ptr<uchar>(u)[v*3+2];

                point_cloud->points.push_back( pcl_point_world );
            }
    }
    point_cloud->is_dense = false;
    pcl::io::savePCDFileBinary("../map.pcd", *point_cloud );

    
    return 0;
}
