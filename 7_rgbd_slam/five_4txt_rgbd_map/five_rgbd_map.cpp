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
#include <pcl/features/normal_3d.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/io.h>
#include <boost/format.hpp>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;

// image2PonitCloud 将rgb图转换为点云
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr image2PointCloud(cv::Mat &rgb, cv::Mat &depth);

int main(int argc, char **argv){
    vector<cv::Mat> rgbImgs, depthImgs;
    cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
    cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();
    boost::format fmt( "../%s/%d.%s" ); 
    for ( int i=0; i<5; i++ ){
        rgbImgs.push_back( cv::imread((fmt%"color"%(i+1)%"png").str()));
        depthImgs.push_back( cv::imread((fmt%"depth"%(i+1)%"pgm").str(), -1)); 
    }

    std::vector<cv::KeyPoint> kp1, kp2;
    detector->detect(rgbImgs[0], kp1);
    detector->detect(rgbImgs[1], kp2);
    cv::Mat imgShow;

    cv::drawKeypoints(rgbImgs[0], kp1, imgShow, cv::Scalar::all(-1), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
    cv::namedWindow("keypoints", CV_WINDOW_NORMAL);
    cv::imshow("keypoints", imgShow);
    cv::waitKey(0);

    cv::Mat desp1, desp2;
    descriptor->compute(rgbImgs[0], kp1, desp1);
    descriptor->compute(rgbImgs[1], kp2, desp2);

    std::vector<cv::DMatch> matches;
    cv::BFMatcher matcher;
    matcher.match(desp1, desp2, matches);
    cout << "Find total " << matches.size() << " matches." << endl;

    cv::Mat imgMatches;
    cv::drawMatches(rgbImgs[0], kp1, rgbImgs[1], kp2, matches, imgMatches);
    cv::imshow("matches", imgMatches);
    cv::waitKey(0);
    return 0;
}

//     cv::Mat rgb1 = cv::imread("../rgb1.png");
//     cv::Mat rgb2 = cv::imread("../rgb2.png");
//     cv::Mat depth1 = cv::imread("../depth1.png", -1);
//     cv::Mat depth2 = cv::imread("../depth2.png", -1);






//     std::vector<cv::DMatch> goodMatches;
//     double minDis = 9999;
//     for (size_t i = 0; i < matches.size(); i++){
//         if (matches[i].distance < minDis)
//             minDis = matches[i].distance;
//     }
//     cout << "min dis = " << minDis << endl;

//     for (size_t i = 0; i < matches.size(); i++){
//         if (matches[i].distance < 10 * minDis)
//             goodMatches.push_back(matches[i]);
//     }
//     cout << "good matches=" << goodMatches.size() << endl;

//     cv::drawMatches(rgb1, kp1, rgb2, kp2, goodMatches, imgMatches);
//     cv::imshow("good matches", imgMatches);
//     cv::waitKey(0);

//     // 调用cv::solvePnPRansac()函数计算两张图像间的运动关系
//     // 第一个张图像的三维点
//     std::vector<cv::Point3f> pts_obj;
//     // 第二个张图像的特征点
//     std::vector<cv::Point2f> pts_img;

//     for (size_t i = 0; i < goodMatches.size(); i++){
//         // query 是第一个张图像, train 是第二个张图像
//         cv::Point2f p = kp1[goodMatches[i].queryIdx].pt;
//         // 获取深度d, y是图像的行, x是图像的列
//         ushort d = depth1.ptr<ushort>(int(p.y))[int(p.x)];
//         if (d == 0)
//             continue;
//         // 将(u,v,d)转成(x,y,z)
//         cv::Point3f pd;
//         pd.z = double(d) / 1000.0;
        
//         // 相机内参
//         const float cx = 325.5;
//         const float cy = 253.5;
//         const float fx = 518.0;
//         const float fy = 519.0;

//         pd.x = (p.x - cx) * pd.z / fx;
//         pd.y = (p.y - cy) * pd.z / fy;

//         // 第一张图像的特征点对应的三维点（和第二张图像匹配）
//         pts_obj.push_back(pd);
//         // 第二张图像的特征点（和第一张图像匹配）
//         pts_img.push_back(cv::Point2f(kp2[goodMatches[i].trainIdx].pt));
//     }

//     double camera_matrix[3][3] = {
//         {518.0, 0,     325.5},
//         {0,     519.0, 253.5},
//         {0,     0,     1}};

//     // 构建相机内参矩阵
//     cv::Mat cameraMatrix(3, 3, CV_64F, camera_matrix);
//     cv::Mat rvec, tvec, inliers;
//     cv::solvePnPRansac(pts_obj, pts_img, cameraMatrix, cv::Mat(), rvec, tvec, false, 100, 1.0, 0.99, inliers);

//     cout << "inliers: " << inliers.rows << endl;
//     cout << "R=" << rvec << endl;
//     cout << "t=" << tvec << endl;


//     vector< cv::DMatch > matchesShow;
//     for (size_t i=0; i<inliers.rows; i++){
//         matchesShow.push_back( goodMatches[inliers.ptr<int>(i)[0]] );    
//     }
//     cv::drawMatches(rgb1, kp1, rgb2, kp2, matchesShow, imgMatches );
//     cv::imshow( "inlier matches", imgMatches );
//     cv::waitKey( 0 );

//     // 旋转向量转旋转矩阵
//     cv::Mat R;
//     cv::Rodrigues(rvec, R);
//     Eigen::Matrix3d r;
//     cv::cv2eigen(R, r);

//     // 旋转矩阵和平移向量转变换矩阵
//     Eigen::Isometry3d transform_matrix = Eigen::Isometry3d::Identity();
//     transform_matrix.pretranslate(Eigen::Vector3d(tvec.at<double>(0, 0), tvec.at<double>(0, 0), tvec.at<double>(0, 0)));
//     transform_matrix.rotate(r);
//     std::cout << "Transform matrix = \n" << transform_matrix.matrix() << std::endl;

//     // 生成第一对rgbd图像的点云
//     pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud1 = image2PointCloud(rgb1, depth1);

//     pcl::io::savePCDFile("../cloud1.pcd", *cloud1);
//     cout << "cloud1 saved" << endl;

//     // 生成第二对rgbd图像的点云
//     pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud2 = image2PointCloud(rgb2, depth2);

//     pcl::io::savePCDFile("../cloud2.pcd", *cloud1);
//     cout << "cloud2 saved" << endl;

//     // 第一对、第二对rgbd图像的点云变换位姿后叠加
//     pcl::PointCloud<pcl::PointXYZRGBA>::Ptr output1(new pcl::PointCloud<pcl::PointXYZRGBA>());
//     pcl::PointCloud<pcl::PointXYZRGBA>::Ptr spft(new pcl::PointCloud<pcl::PointXYZRGBA>());
//     pcl::transformPointCloud(*cloud1, *output1, transform_matrix.matrix());
//     *spft = *cloud2 + *output1;
//     pcl::io::savePCDFile("../spft.pcd", *spft);
//     cout << "spft saved" << endl;

//     // 第一对、第二对rgbd图像的点云变换位姿后叠加
//     pcl::PointCloud<pcl::PointXYZRGBA>::Ptr output2(new pcl::PointCloud<pcl::PointXYZRGBA>());
//     pcl::PointCloud<pcl::PointXYZRGBA>::Ptr fpst(new pcl::PointCloud<pcl::PointXYZRGBA>());
//     pcl::transformPointCloud(*cloud2, *output2, transform_matrix.matrix());
//     *fpst = *cloud1 + *output2;
//     pcl::io::savePCDFile("../fpst.pcd", *fpst);
//     cout << "fpst saved" << endl;

//     // //创建点云渲染句柄
//     // pcl::visualization::CloudViewer viewer("Cloud Viewer"); //创建viewer对象
//     // //showCloud函数是同步的，在此处等待直到渲染显示为止
//     // viewer.showCloud(cloud1);
//     // while (!viewer.wasStopped()){
//     // }

//     // pcl::visualization::PCLVisualizer viewer ("Matrix transformation example");
//     // viewer.addPointCloud (cloud2, "original_cloud");
//     // viewer.addPointCloud (output1, "transformed_cloud");
//     // viewer.addCoordinateSystem (1.0, 0); 
//     // viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
//     // viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud");
//     // while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
//     //     viewer.spinOnce ();
//     // }



pcl::PointCloud<pcl::PointXYZRGBA>::Ptr image2PointCloud(cv::Mat &rgb, cv::Mat &depth){
    // 相机内参
    const float cx = 325.5;
    const float cy = 253.5;
    const float fx = 518.0;
    const float fy = 519.0;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    for (int y = 0; y < depth.rows; y++)
       for (int x = 0; x < depth.cols; x++)
       {
           // 深度图中坐标(m,n)处的值是彩色图像像素坐标(m,n)处的深度值
           // ushort d = depth.ptr<ushort>(m)[n];
           // 获取深度d, y是图像的行, x是图像的列
           ushort d = depth.ptr<ushort>(int(y))[int(x)];
           if (d == 0)
               continue;
           pcl::PointXYZRGBA point;
           // 计算这个点的空间坐标
           point.z = double(d) / 1000.0;
           point.x = (x - cx) * point.z / fx;
           point.y = -(y - cy) * point.z / fy;

           // 从rgb图像中获取它的颜色
           // rgb是三通道的BGR格式图，所以按下面的顺序获取颜色
           point.b = rgb.ptr<uchar>(y)[x * 3];
           point.g = rgb.ptr<uchar>(y)[x * 3 + 1];
           point.r = rgb.ptr<uchar>(y)[x * 3 + 2];

           // 把p加入到点云中
           cloud->points.push_back(point);
       }
   // 设置并保存点云
   cloud->height = 1;
   cloud->width = cloud->points.size();
   cloud->is_dense = false;
   return cloud;
}
