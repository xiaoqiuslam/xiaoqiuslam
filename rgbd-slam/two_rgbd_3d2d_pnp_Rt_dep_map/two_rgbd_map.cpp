#include <iostream>
#include <fstream>
#include <vector>
#include <map>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>

using namespace std;

// 类型定义
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;

// image2PonitCloud 将rgb图转换为点云
PointCloud::Ptr image2PointCloud(cv::Mat &rgb, cv::Mat &depth);

int main(int argc, char **argv)
{
    cv::Mat rgb1 = cv::imread("../data/rgb1.png");
    cv::Mat rgb2 = cv::imread("../data/rgb2.png");
    cv::Mat depth1 = cv::imread("../data/depth1.png", -1);
    cv::Mat depth2 = cv::imread("../data/depth2.png", -1);

    cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
    cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();

    std::vector<cv::KeyPoint> kp1, kp2;
    detector->detect(rgb1, kp1);
    detector->detect(rgb2, kp2);
    cout << "Key points of two images: " << kp1.size() << ", " << kp2.size() << endl;
    cv::Mat imgShow;
    cv::drawKeypoints(rgb1, kp1, imgShow, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    cv::imshow("keypoints", imgShow);
    cv::waitKey(0);
    cv::Mat desp1, desp2;
    descriptor->compute(rgb1, kp1, desp1);
    descriptor->compute(rgb2, kp2, desp2);
    std::vector<cv::DMatch> matches;
    cv::BFMatcher matcher;
    matcher.match(desp1, desp2, matches);
    cout << "Find total " << matches.size() << " matches." << endl;
    cv::Mat imgMatches;
    cv::drawMatches(rgb1, kp1, rgb2, kp2, matches, imgMatches);
    cv::imshow("matches", imgMatches);
    cv::waitKey(0);
    std::vector<cv::DMatch> goodMatches;
    double minDis = 9999;
    for (size_t i = 0; i < matches.size(); i++)
    {
        if (matches[i].distance < minDis)
            minDis = matches[i].distance;
    }
    cout << "min dis = " << minDis << endl;

    for (size_t i = 0; i < matches.size(); i++)
    {
        if (matches[i].distance < 10 * minDis)
            goodMatches.push_back(matches[i]);
    }
    cout << "good matches=" << goodMatches.size() << endl;
    cv::drawMatches(rgb1, kp1, rgb2, kp2, goodMatches, imgMatches);
    cv::imshow("good matches", imgMatches);
    cv::waitKey(0);

    // 计算图像间的运动关系cv::solvePnPRansac()为调用此函数准备必要的参数
    // 第一个帧的三维点
    std::vector<cv::Point3f> pts_obj;
    // 第二个帧的图像点
    std::vector<cv::Point2f> pts_img;
    for (size_t i = 0; i < goodMatches.size(); i++)
    {
        // query 是第一帧, train 是第二帧
        cv::Point2f p = kp1[goodMatches[i].queryIdx].pt;
        // 获取dx,y是行，x是列
        ushort d = depth1.ptr<ushort>(int(p.y))[int(p.x)];
        if (d == 0)
            continue;
        // 将(u,v,d)转成(x,y,z)
        cv::Point3f pd;
        pd.z = double(d) / 1000.0;
        
        // 相机内参
        const float cx = 325.5;
        const float cy = 253.5;
        const float fx = 518.0;
        const float fy = 519.0;

        pd.x = (p.x - cx) * pd.z / fx;
        pd.y = (p.y - cy) * pd.z / fy;

        pts_obj.push_back(pd);

        pts_img.push_back(cv::Point2f(kp2[goodMatches[i].trainIdx].pt));
    }

    double camera_matrix[3][3] = {
        {518.0, 0, 325.5},
        {0, 519.0, 253.5},
        {0, 0, 1}};

    // 构建相机矩阵
    cv::Mat cameraMatrix(3, 3, CV_64F, camera_matrix);
    cv::Mat rvec, tvec, inliers;
    cv::solvePnPRansac(pts_obj, pts_img, cameraMatrix, cv::Mat(), rvec, tvec, false, 100, 1.0, 0.99, inliers);

    cout << "inliers: " << inliers.rows << endl;
    cout << "R=" << rvec << endl;
    cout << "t=" << tvec << endl;

    // 将旋转向量转化为旋转矩阵
    cv::Mat R;
    cv::Rodrigues(rvec, R);
    Eigen::Matrix3d r;
    cv::cv2eigen(R, r);

    // 将平移向量和旋转矩阵转换成变换矩阵
    Eigen::Isometry3d transform_matrix = Eigen::Isometry3d::Identity();
    transform_matrix.rotate(r);
    transform_matrix.pretranslate(Eigen::Vector3d(tvec.at<double>(0, 0), tvec.at<double>(0, 0), tvec.at<double>(0, 0)));
    std::cout << "Transform matrix = \n" << transform_matrix.matrix() << std::endl;

    // 第一对rgbd图像的点云
    PointCloud::Ptr cloud1 = image2PointCloud(rgb1, depth1);

    // 第二对rgbd图像的点云
    PointCloud::Ptr cloud2 = image2PointCloud(rgb2, depth2);

    // 第一对、第二对rgbd图像的点云变换位姿后叠加
    PointCloud::Ptr output1(new PointCloud());
    pcl::transformPointCloud(*cloud1, *output1, transform_matrix.matrix());
    *cloud2 += *output1;
    pcl::io::savePCDFile("../data/t1plus2.pcd", *output1);
    cout << "t1plus2 saved" << endl;

    return 0;
}


PointCloud::Ptr image2PointCloud(cv::Mat &rgb, cv::Mat &depth){

    // 相机内参
    const float cx = 325.5;
    const float cy = 253.5;
    const float fx = 518.0;
    const float fy = 519.0;

    PointCloud::Ptr cloud(new PointCloud);

    for (int m = 0; m < depth.rows; m++)
       for (int n = 0; n < depth.cols; n++)
       {
           // 获取深度图中(m,n)处的值
           ushort d = depth.ptr<ushort>(m)[n];
           // d 可能没有值，若如此，跳过此点
           if (d == 0)
               continue;
           // d 存在值，则向点云增加一个点
           PointT p;
           // 计算这个点的空间坐标
           p.z = double(d) / 1000.0;
           p.x = (n - cx) * p.z / fx;
           p.y = -(m - cy) * p.z / fy;

           // 从rgb图像中获取它的颜色
           // rgb是三通道的BGR格式图，所以按下面的顺序获取颜色
           p.b = rgb.ptr<uchar>(m)[n * 3];
           p.g = rgb.ptr<uchar>(m)[n * 3 + 1];
           p.r = rgb.ptr<uchar>(m)[n * 3 + 2];

           // 把p加入到点云中
           cloud->points.push_back(p);
       }
   // 设置并保存点云
   cloud->height = 1;
   cloud->width = cloud->points.size();
   cloud->is_dense = false;
   return cloud;
}
