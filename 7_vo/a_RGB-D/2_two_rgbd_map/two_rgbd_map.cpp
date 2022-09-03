#include <iostream>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>

const double fx = 518.0;
const double fy = 519.0;
const double cx = 325.5;
const double cy = 253.5;
const double factor = 1000.0;

cv::Mat result_rvec, result_tvec;
int result_inliers;

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr image2PointCloud( cv::Mat& rgb, cv::Mat& depth)
{
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud ( new pcl::PointCloud<pcl::PointXYZRGBA> );

    for (int u = 0; u < depth.rows; u++)
        for (int v =0; v < depth.cols; v++)
        {
            ushort d = depth.ptr<ushort>(u)[v];
            if (d == 0)
                continue;

            pcl::PointXYZRGBA point;

            point.z = double(d) / factor;
            point.x = (v - cx) * point.z / fx;
            point.y = (u - cy) * point.z / fy;

            point.b = rgb.ptr<uchar>(u)[v*3];
            point.g = rgb.ptr<uchar>(u)[v*3+1];
            point.r = rgb.ptr<uchar>(u)[v*3+2];

            cloud->points.push_back(point);
        }
    cloud->height = 1;
    cloud->width = cloud->points.size();
    cloud->is_dense = false;
    return cloud;
}

void estimateMotion( std::vector<cv::KeyPoint>& lastFrame_kp, cv::Mat& lastFrame_desp, cv::Mat& lastFrame_depth, std::vector<cv::KeyPoint>& currFrame_kp, cv::Mat& currFrame_desp)
{
    std::vector< cv::DMatch > matches;
    cv::BFMatcher matcher;
    matcher.match( lastFrame_desp, currFrame_desp, matches );

    std::vector< cv::DMatch > goodMatches;
    double minDis = 9999;
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

    if (goodMatches.size() <= 5)
    {
        result_inliers = -1;
    }

    std::vector<cv::Point3f> pts_obj;
    std::vector< cv::Point2f > pts_img;
    for (size_t i=0; i<goodMatches.size(); i++)
    {
        cv::Point2f p = lastFrame_kp[goodMatches[i].queryIdx].pt;
        ushort d = lastFrame_depth.ptr<ushort>( int(p.y) )[ int(p.x) ];
        if (d == 0)
            continue;
        pts_img.push_back( cv::Point2f( currFrame_kp[goodMatches[i].trainIdx].pt ) );

        cv::Point3f pt ( p.x, p.y, d );
        pt.z = double( d) / factor;
        pt.x = ( p.x - cx) * pt.z / fx;
        pt.y = ( p.y - cy) * pt.z / fy;

        pts_obj.push_back( pt );
    }

    if (pts_obj.size() ==0 || pts_img.size()==0)
    {
        result_inliers = -1;
    }

    double camera_matrix_data[3][3] = {
            {fx, 0, cx},
            {0, fy, cy},
            {0, 0, 1}
    };

    cv::Mat cameraMatrix( 3, 3, CV_64F, camera_matrix_data );
    cv::Mat rvec, tvec, inliers;
    cv::solvePnPRansac( pts_obj, pts_img, cameraMatrix, cv::Mat(), rvec, tvec, false, 100, 1.0, 0.99, inliers );

    result_rvec = rvec;
    result_tvec = tvec;
    result_inliers = inliers.rows;
}

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr joinPointCloud( pcl::PointCloud<pcl::PointXYZRGBA>::Ptr original,  cv::Mat& currFrame_rgb, cv::Mat& currFrame_depth, Eigen::Isometry3d T)
{
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr newCloud = image2PointCloud( currFrame_rgb, currFrame_depth);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr output (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::transformPointCloud( *original, *output, T.matrix() );
    // 函数的作用是通过转换矩阵transform将original转换后存到output中保存。
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

    cv::Mat lastFrame_rgb, lastFrame_depth;
    cv::Mat currFrame_rgb, currFrame_depth;

    cv::Mat lastFrame_desp;
    cv::Mat currFrame_desp;

    std::vector<cv::KeyPoint> lastFrame_kp;
    std::vector<cv::KeyPoint> currFrame_kp;

    pcl::visualization::CloudViewer viewer("viewer");

    lastFrame_rgb = cv::imread("/home/q/rgbd_slam/3_two_rgbd_map/rgb1.png");
    lastFrame_depth = cv::imread("/home/q/rgbd_slam/3_two_rgbd_map/depth1.png", -1 );
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud = image2PointCloud( lastFrame_rgb, lastFrame_depth);

    cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
    cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();

    detector->detect( lastFrame_rgb, lastFrame_kp );
    descriptor->compute( lastFrame_rgb, lastFrame_kp, lastFrame_desp );

    currFrame_rgb = cv::imread("/home/q/rgbd_slam/3_two_rgbd_map/rgb2.png");
    currFrame_depth = cv::imread("/home/q/rgbd_slam/3_two_rgbd_map/depth2.png", -1 );

    detector->detect( currFrame_rgb, currFrame_kp );
    descriptor->compute( currFrame_rgb, currFrame_kp, currFrame_desp );


    estimateMotion(lastFrame_kp, lastFrame_desp, lastFrame_depth, currFrame_kp, currFrame_desp);


    cv::Mat R;
    cv::Rodrigues( result_rvec, R );
    Eigen::Matrix3d r;
    cv::cv2eigen(R, r);

    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();

    Eigen::AngleAxisd angle(r);
    T = angle;
    T(0,3) = result_tvec.at<double>(0,0);
    T(1,3) = result_tvec.at<double>(1,0);
    T(2,3) = result_tvec.at<double>(2,0);

    cout<<"T\n"<<T.matrix()<<endl;
//        0.999324 -0.0199144  0.0308938  0.0252744
//        0.0191543   0.999511  0.0247063 0.00722174
//        -0.0313707 -0.0240979   0.999217  0.0149321
//        0          0          0          1


    cloud = joinPointCloud( cloud, currFrame_rgb, currFrame_depth, T);


    while (!viewer.wasStopped())
    {
        viewer.showCloud( cloud );
    }

    pcl::io::savePCDFile( "../result.pcd", *cloud );
    return 0;
}