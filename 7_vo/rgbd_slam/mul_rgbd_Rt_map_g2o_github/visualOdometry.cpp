#include <iostream>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>

#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_factory.h>
#include <g2o/core/optimization_algorithm_levenberg.h>

using namespace std;

double fx = 518.0;
double fy = 519.0;
double cx = 325.5;
double cy = 253.5;
double scale = 1000.0;

// 帧结构
struct FRAME
{
    cv::Mat rgb, depth; //该帧对应的彩色图与深度图
    cv::Mat desp;       //特征描述子
    vector<cv::KeyPoint> kp; //关键点
};

// PnP 结果
struct RESULT_OF_PNP
{
    cv::Mat rvec, tvec;
    int inliers;
};

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr image2PointCloud( cv::Mat& rgb, cv::Mat& depth)
{
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud ( new pcl::PointCloud<pcl::PointXYZRGBA> );

    for (int m = 0; m < depth.rows; m+=2)
        for (int n=0; n < depth.cols; n+=2)
        {
            // 获取深度图中(m,n)处的值
            ushort d = depth.ptr<ushort>(m)[n];
            // d 可能没有值，若如此，跳过此点
            if (d == 0)
                continue;
            // d 存在值，则向点云增加一个点
            pcl::PointXYZRGBA p;

            // 计算这个点的空间坐标
            p.z = double(d) / scale;
            p.x = (n - cx) * p.z / fx;
            p.y = (m - cy) * p.z / fy;

            // 从rgb图像中获取它的颜色
            // rgb是三通道的BGR格式图，所以按下面的顺序获取颜色
            p.b = rgb.ptr<uchar>(m)[n*3];
            p.g = rgb.ptr<uchar>(m)[n*3+1];
            p.r = rgb.ptr<uchar>(m)[n*3+2];

            // 把p加入到点云中
            cloud->points.push_back( p );
        }
    // 设置并保存点云
    cloud->height = 1;
    cloud->width = cloud->points.size();
    cloud->is_dense = false;

    return cloud;
}





// estimateMotion 计算两个帧之间的运动
// 输入：帧1和帧2
// 输出：rvec 和 tvec
RESULT_OF_PNP estimateMotion( FRAME& frame1, FRAME& frame2)
{
    vector< cv::DMatch > matches;
    cv::BFMatcher matcher;
    matcher.match( frame1.desp, frame2.desp, matches );

    RESULT_OF_PNP result;
    vector< cv::DMatch > goodMatches;
    double minDis = 9999;
    // 筛选good match的倍数
    double good_match_threshold = 10;
    for ( size_t i=0; i<matches.size(); i++ )
    {
        if ( matches[i].distance < minDis )
            minDis = matches[i].distance;
    }

    cout<<"min dis = "<<minDis<<endl;
    if ( minDis < 15 )
        minDis = 15;

    for ( size_t i=0; i<matches.size(); i++ )
    {
        if (matches[i].distance < good_match_threshold*minDis )
            goodMatches.push_back( matches[i] );
    }

    cout<<"good matches: "<<goodMatches.size()<<endl;

    if (goodMatches.size() <= 5)
    {
        result.inliers = -1;
        return result;
    }

    // 第一个帧的三维点
    vector<cv::Point3f> pts_obj;
    // 第二个帧的图像点
    vector< cv::Point2f > pts_img;

    // 相机内参
    for (size_t i=0; i<goodMatches.size(); i++)
    {
        // query 是第一个, train 是第二个
        cv::Point2f p = frame1.kp[goodMatches[i].queryIdx].pt;
        // 获取d是要小心！x是向右的，y是向下的，所以y才是行，x是列！
        ushort d = frame1.depth.ptr<ushort>( int(p.y) )[ int(p.x) ];
        if (d == 0)
            continue;
        pts_img.push_back( cv::Point2f( frame2.kp[goodMatches[i].trainIdx].pt ) );

        // 将(u,v,d)转成(x,y,z)
        cv::Point3f pt ( p.x, p.y, d );
        pt.z = double( d) / scale;
        pt.x = ( p.x - cx) * pt.z / fx;
        pt.y = ( p.y - cy) * pt.z / fy;

        pts_obj.push_back( pt );
    }

    if (pts_obj.size() ==0 || pts_img.size()==0)
    {
        result.inliers = -1;
        return result;
    }

    double camera_matrix_data[3][3] = {
            {fx, 0, cx},
            {0, fy, cy},
            {0, 0, 1}
    };

    // 构建相机矩阵
    cv::Mat cameraMatrix( 3, 3, CV_64F, camera_matrix_data );
    cv::Mat rvec, tvec, inliers;
    // 求解pnp
    cv::solvePnPRansac( pts_obj, pts_img, cameraMatrix, cv::Mat(), rvec, tvec, false, 100, 1.0, 0.99, inliers );

    result.rvec = rvec;
    result.tvec = tvec;
    result.inliers = inliers.rows;
    return result;
}


// joinPointCloud
// 输入：原始点云，新来的帧以及它的位姿
// 输出：将新来帧加到原始帧后的图像
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr joinPointCloud( pcl::PointCloud<pcl::PointXYZRGBA>::Ptr original, FRAME& newFrame, Eigen::Isometry3d T)
{
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr newCloud = image2PointCloud( newFrame.rgb, newFrame.depth);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr output (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::transformPointCloud( *original, *output, T.matrix() );
    *newCloud += *output;

    // Voxel grid 滤波降采样
    static pcl::VoxelGrid<pcl::PointXYZRGBA> voxel;

    // 点云分辨率
    double gridsize = 0.01;
    voxel.setLeafSize( gridsize, gridsize, gridsize );
    voxel.setInputCloud( newCloud );
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tmp( new pcl::PointCloud<pcl::PointXYZRGBA> );
    voxel.filter( *tmp );
    return tmp;
}


int main( int argc, char** argv){
    int startIndex  =   1;
    int endIndex    =   700;
    // 当前索引
    int currIndex = startIndex;
    // 读取上一帧
    FRAME lastFrame;
    stringstream ss;
    ss<< "/media/q/q/rgbd-slam-tutorial/mul_rgbd_Rt_map_g2o/rgb_png/" <<currIndex<< ".png";
    string filename;
    ss>>filename;
    lastFrame.rgb = cv::imread( filename );

    ss.clear();
    filename.clear();
    ss<<"/media/q/q/rgbd-slam-tutorial/mul_rgbd_Rt_map_g2o/depth_png/"<<currIndex<< ".png";
    ss>>filename;

    lastFrame.depth = cv::imread( filename, -1 );


    cv::Ptr<cv::FeatureDetector> _detector = cv::ORB::create();
    cv::Ptr<cv::DescriptorExtractor> _descriptor = cv::ORB::create();

    _detector->detect( lastFrame.rgb, lastFrame.kp );
    _descriptor->compute( lastFrame.rgb, lastFrame.kp, lastFrame.desp );

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud = image2PointCloud( lastFrame.rgb, lastFrame.depth);
    pcl::visualization::CloudViewer viewer("viewer");


    // 最大运动误差
    double max_norm = 0.3;
    for (currIndex = startIndex + 1; currIndex<endIndex; currIndex++ ){
        // 读取当前帧
        FRAME currFrame;
        stringstream ss;
        ss<< "/media/q/q/rgbd-slam-tutorial/mul_rgbd_Rt_map_g2o/rgb_png/" <<currIndex<< ".png";
        string filename;
        ss>>filename;
        currFrame.rgb = cv::imread( filename );

        ss.clear();
        filename.clear();
        ss<<"/media/q/q/rgbd-slam-tutorial/mul_rgbd_Rt_map_g2o/depth_png/"<<currIndex<< ".png";
        ss>>filename;

        currFrame.depth = cv::imread( filename, -1 );

        cv::Ptr<cv::FeatureDetector> _detector = cv::ORB::create();
        cv::Ptr<cv::DescriptorExtractor> _descriptor = cv::ORB::create();
        _detector->detect( currFrame.rgb, currFrame.kp );
        _descriptor->compute( currFrame.rgb, currFrame.kp, currFrame.desp );

        // 计算上一帧和当前帧之间的位姿变换
        RESULT_OF_PNP result = estimateMotion( lastFrame, currFrame);
        //　最小内点>上一帧和当前帧匹配的特征点不够放弃该帧
        int min_inliers = 4;
        if ( result.inliers < min_inliers )
            continue;
        // 计算运动范围是否太大
        double norm = fabs(min(cv::norm(result.rvec), 2*M_PI-cv::norm(result.rvec))) + fabs(cv::norm(result.tvec));
        cout<<"norm = "<<norm<<endl;
        if ( norm >= max_norm )
            continue;

        cv::Mat R;
        cv::Rodrigues( result.rvec, R );
        Eigen::Matrix3d r;
        for ( int i=0; i<3; i++ )
            for ( int j=0; j<3; j++ )
                r(i,j) = R.at<double>(i,j);


        // 将平移向量和旋转矩阵转换成变换矩阵
        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();

        Eigen::AngleAxisd angle(r);
        T = angle;
        T(0,3) = result.tvec.at<double>(0,0);
        T(1,3) = result.tvec.at<double>(1,0);
        T(2,3) = result.tvec.at<double>(2,0);
        cout<<"T="<<T.matrix()<<endl;


        cloud = joinPointCloud( cloud, currFrame, T);
        
        viewer.showCloud( cloud );

        lastFrame = currFrame;
    }

    pcl::io::savePCDFile( "../result.pcd", *cloud );
    return 0;
}