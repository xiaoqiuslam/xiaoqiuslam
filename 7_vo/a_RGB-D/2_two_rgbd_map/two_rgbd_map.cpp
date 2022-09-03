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
#include <pcl/features/normal_3d.h>
#include <pcl/io/io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr image2PointCloud(cv::Mat &rgb, cv::Mat &depth);

// 相机内参
const double factor = 1000;
const float cx = 325.5;
const float cy = 253.5;
const float fx = 518.0;
const float fy = 519.0;

int main(int argc, char **argv){

    cv::Mat rgb1 = cv::imread("../rgb1.png");
    cv::Mat rgb2 = cv::imread("../rgb2.png");
    cv::Mat depth1 = cv::imread("../depth1.png", -1);
    cv::Mat depth2 = cv::imread("../depth2.png", -1);

    cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
    cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();

    std::vector<cv::KeyPoint> kp1, kp2;
    detector->detect(rgb1, kp1);
    detector->detect(rgb2, kp2);
    cout << "Key points of two images: " << kp1.size() << ", " << kp2.size() << endl;

    cv::Mat imgShow_rgb1;
    cv::drawKeypoints(rgb1, kp1, imgShow_rgb1, cv::Scalar::all(-1), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
    cv::namedWindow("kp1", cv::WINDOW_NORMAL);
    cv::imshow("kp1", imgShow_rgb1);
    cv::waitKey(0);

    cv::Mat imgShow_rgb2;
    cv::drawKeypoints(rgb2, kp2, imgShow_rgb2, cv::Scalar::all(-1), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
    cv::namedWindow("kp2", cv::WINDOW_NORMAL);
    cv::imshow("kp2", imgShow_rgb2);
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
    for (size_t i = 0; i < matches.size(); i++){
        if (matches[i].distance < minDis)
            minDis = matches[i].distance;
    }
    cout << "min distance descriptor: " << minDis << endl;

    for (size_t i = 0; i < matches.size(); i++){
        if (matches[i].distance < 10 * minDis)
            goodMatches.push_back(matches[i]);
    }
    cout << "good matches descriptor:" << goodMatches.size() << endl;
    cv::drawMatches(rgb1, kp1, rgb2, kp2, goodMatches, imgMatches);
    cv::imshow("good matches", imgMatches);
    cv::waitKey(0);

    // cv::solvePnPRansac()计算两张图像变换矩阵（旋转和平移）
    // 第一个张图像的三维路标点
    std::vector<cv::Point3f> pts_obj;
    // 第二个张图像的二维特征点
    std::vector<cv::Point2f> pts_img;
    for (size_t i = 0; i < goodMatches.size(); i++){
        // queryIdx 是第一个张图像特征点的索引, trainIdx 是第二张图像特征点的索引
        cv::Point2f p = kp1[goodMatches[i].queryIdx].pt;
        // 获取深度d, y是图像的行, x是图像的列
        ushort d = depth1.ptr<ushort>(int(p.y))[int(p.x)];
        if (d == 0)
            continue;

        // 将(p.y,p.x,d)转成(x,y,z)
        cv::Point3f pd;
        pd.z = double(d) / factor;
        pd.x = (p.x - cx) * pd.z / fx;
        pd.y = (p.y - cy) * pd.z / fy;

        // 第一张图像的二维特征点对应的三维路标点（和第二张图像的二维特征点匹配）
        pts_obj.push_back(pd);
        // 第二张图像的特征点（和第一张图像的三维路标点匹配）
        pts_img.push_back(cv::Point2f(kp2[goodMatches[i].trainIdx].pt));
    }

    double camera_matrix[3][3] = {
        {fx, 0,     cx},
        {0,     fy, cy},
        {0,     0,     1}};

    // 构建相机内参矩阵
    cv::Mat cameraMatrix(3, 3, CV_64F, camera_matrix);
    cv::Mat rvec, tvec, inliers;
    cv::solvePnPRansac(pts_obj, pts_img, cameraMatrix, cv::Mat(), rvec, tvec, false, 100, 1.0, 0.99, inliers);

    cout << "inliers: " << inliers.rows << endl;
    cout << "R: " << rvec << endl;
    cout << "t: " << tvec << endl;

    std::vector< cv::DMatch > matchesShow;
    for (size_t i=0; i<inliers.rows; i++){
        matchesShow.push_back(goodMatches[inliers.ptr<int>(i)[0]]);
    }
    cv::drawMatches(rgb1, kp1, rgb2, kp2, matchesShow, imgMatches );
    cv::imshow( "inlier matches", imgMatches );
    cv::waitKey( 0 );

    // 旋转向量转旋转矩阵
    cv::Mat R;
    cv::Rodrigues(rvec, R);
    Eigen::Matrix3d r;
    cv::cv2eigen(R, r);

    // 旋转矩阵和平移向量组装变换矩阵
    Eigen::Isometry3d transform_matrix_rotate = Eigen::Isometry3d::Identity();
    transform_matrix_rotate.pretranslate(Eigen::Vector3d(tvec.at<double>(0, 0), tvec.at<double>(0, 1), tvec.at<double>(0, 2)));
    transform_matrix_rotate.rotate(r);
    std::cout << "transform_matrix_rotate matrix: \n" << transform_matrix_rotate.matrix() << std::endl;

    Eigen::Isometry3d transform_matrix_AngleAxisd = Eigen::Isometry3d::Identity();
    Eigen::AngleAxisd angle(r);
    Eigen::Translation<double,3> trans(tvec.at<double>(0,0), tvec.at<double>(0,1), tvec.at<double>(0,2));
    transform_matrix_AngleAxisd = angle;
    transform_matrix_AngleAxisd(0,3) = tvec.at<double>(0,0);
    transform_matrix_AngleAxisd(1,3) = tvec.at<double>(0,1);
    transform_matrix_AngleAxisd(2,3) = tvec.at<double>(0,2);
    std::cout << "transform_matrix_AngleAxisd matrix: \n" << transform_matrix_AngleAxisd.matrix() << std::endl;

    // 生成第一对rgbd图像的点云
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud1 = image2PointCloud(rgb1, depth1);
    // Voxel grid 滤波降采样
    static pcl::VoxelGrid<pcl::PointXYZRGBA> voxel;
    voxel.setLeafSize( 0.01, 0.01, 0.01 );
    voxel.setInputCloud( cloud1 );
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tmp( new  pcl::PointCloud<pcl::PointXYZRGBA>() );
    voxel.filter( *tmp );

    pcl::io::savePCDFile("../cloud1.pcd", *tmp);
    cout << "cloud1 saved" << endl;

    // 生成第二对rgbd图像的点云
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud2 = image2PointCloud(rgb2, depth2);
    voxel.setInputCloud( cloud2 );
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tmp2( new  pcl::PointCloud<pcl::PointXYZRGBA>() );
    voxel.filter( *tmp2 );
    pcl::io::savePCDFile("../cloud2.pcd", *tmp2);
    cout << "cloud2 saved" << endl;

    // 第一对、第二对rgbd图像的点云变换位姿后叠加
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr output1(new pcl::PointCloud<pcl::PointXYZRGBA>());
    pcl::transformPointCloud(*cloud1, *output1, transform_matrix_rotate.matrix());
    *output1 += *cloud2;
    pcl::io::savePCDFile("../output1.pcd", *output1);
    cout << "output1 saved" << endl;

    // 第一对、第二对rgbd图像的点云变换位姿后叠加
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr output2(new pcl::PointCloud<pcl::PointXYZRGBA>());
    pcl::transformPointCloud(*cloud2, *output2, transform_matrix_rotate.matrix());
    *output2 += *cloud1;
    pcl::io::savePCDFile("../output2.pcd", *output2);
    cout << "output2 saved" << endl;

    pcl::visualization::PCLVisualizer viewer("two viewer");
    int v1(0); //设置左右窗口
    int v2(1);

    // (Xmin,Ymin,Xmax,Ymax)设置窗口坐标
    viewer.createViewPort(0.0, 0.0, 0.5, 1, v1);
    viewer.setBackgroundColor(0, 0, 0, v1);

    viewer.createViewPort(0.5, 0.0, 1, 1, v2);
    viewer.setBackgroundColor(0.5, 0.5, 0.5, v2);

    viewer.addPointCloud(output1, "output1");
    viewer.addPointCloud(output2, "output2");

    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }
    return 0;
}


pcl::PointCloud<pcl::PointXYZRGBA>::Ptr image2PointCloud(cv::Mat &rgb, cv::Mat &depth){
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    for (int u = 0; u < depth.rows; u++)
       for (int v = 0; v < depth.cols; v++)
       {
           // 深度图中坐标(u,v)处的值是彩色图像像素坐标(u,v)处的深度值
           // ushort d = depth.ptr<ushort>(u)[v];
           // 获取深度d, u是图像的行, v是图像的列
           ushort d = depth.ptr<ushort>(int(u))[int(v)];
           if (d == 0)
               continue;
           pcl::PointXYZRGBA point;
           // 计算点的三维空间坐标
           point.z = double(d) / 1000.0;
           point.x = (v - cx) * point.z / fx;
           point.y = -(u - cy) * point.z / fy;

           // 从rgb图像中获取它的颜色,rgb是三通道的BGR格式图,按下面的顺序获取颜色
           point.b = rgb.ptr<uchar>(u)[v * 3];
           point.g = rgb.ptr<uchar>(u)[v * 3 + 1];
           point.r = rgb.ptr<uchar>(u)[v * 3 + 2];

           // point加入到点云中
           cloud->points.push_back(point);
       }
   // 设置并保存点云
   cloud->height = 1;
   cloud->width = cloud->points.size();
   cloud->is_dense = false;
   return cloud;
}
