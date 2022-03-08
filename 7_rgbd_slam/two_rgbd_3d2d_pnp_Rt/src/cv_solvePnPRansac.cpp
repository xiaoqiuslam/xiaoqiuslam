#include <iostream>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>

using namespace std;

// 相机内参
const float cx = 325.5;
const float cy = 253.5;
const float fx = 518.0;
const float fy = 519.0;
const float scale = 1000.0;

int main( int argc, char** argv )
{
    // 声明并从data文件夹里读取两个rgb与深度图
    cv::Mat rgb1 = cv::imread( "../data/rgb1.png");
    cv::Mat rgb2 = cv::imread( "../data/rgb2.png");
    cv::Mat depth1 = cv::imread( "../data/depth1.png", -1);
    cv::Mat depth2 = cv::imread( "../data/depth2.png", -1);

    cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
    cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();

    vector< cv::KeyPoint > kp1, kp2; //关键点

    detector->detect( rgb1, kp1 );  //提取关键点
    detector->detect( rgb2, kp2 );

    cout<<"Key points of two images: "<<kp1.size()<<", "<<kp2.size()<<endl;
    
    // 可视化显示关键点
    cv::Mat imgShow;
    cv::drawKeypoints( rgb1, kp1, imgShow, cv::Scalar::all(-1), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
    cv::namedWindow("keypoints", CV_WINDOW_NORMAL);
    cv::imshow( "keypoints", imgShow );
    cv::waitKey(0); //暂停等待一个按键
   
    // 计算描述子
    cv::Mat desp1, desp2;
    descriptor->compute( rgb1, kp1, desp1 );
    descriptor->compute( rgb2, kp2, desp2 );

    // 匹配描述子
    vector< cv::DMatch > matches; 
    cv::BFMatcher matcher;
    matcher.match( desp1, desp2, matches );
    cout<<"Find total "<<matches.size()<<" matches."<<endl;

    // 可视化显示匹配的特征
    cv::Mat imgMatches;
    cv::drawMatches( rgb1, kp1, rgb2, kp2, matches, imgMatches );
    cv::imshow( "matches", imgMatches );
    cv::waitKey( 0 );

    // 筛选匹配，把距离太大的去掉
    // 这里使用的准则是去掉大于四倍最小距离的匹配
    vector< cv::DMatch > goodMatches;
    double minDis = 9999;
    for ( size_t i=0; i<matches.size(); i++ )
    {
        if ( matches[i].distance < minDis )
            minDis = matches[i].distance;
    }
    cout<<"min dis = "<<minDis<<endl;

    for ( size_t i=0; i<matches.size(); i++ )
    {
        if (matches[i].distance < 10*minDis)
            goodMatches.push_back( matches[i] );
    }

    // 显示 good matches
    cout<<"good matches="<<goodMatches.size()<<endl;
    cv::drawMatches( rgb1, kp1, rgb2, kp2, goodMatches, imgMatches );
    cv::imshow( "good matches", imgMatches );
    cv::waitKey(0);

    // 计算图像间的运动关系
    // 关键函数：cv::solvePnPRansac()为调用此函数准备必要的参数
    
    // 第一个帧的三维点
    vector<cv::Point3f> pts_obj;

    // 第二个帧的图像点
    vector< cv::Point2f > pts_img;

    // vector< cv::KeyPoint > kp1, kp2; //关键点
    for (size_t i=0; i<goodMatches.size(); i++)
    {
        // query 是第一帧, train 是第二帧
        cv::Point2f p = kp1[goodMatches[i].queryIdx].pt;

        // 获取d是要小心！x是向右的，y是向下的，所以y才是行，x是列！
        ushort d = depth1.ptr<ushort>( int(p.y) )[ int(p.x) ];
        if (d == 0)
            continue;
        // 将(u,v,d)转成(x,y,z)
        cv::Point3f pt ( p.x, p.y, d );
        cv::Point3f pd ; // 3D 点
        pd.z = float( pt.z ) / scale;
        pd.x = ( pt.x - cx) * pd.z / fx;
        pd.y = ( pt.y - cy) * pd.z / fy;


        pts_obj.push_back( pd );

        pts_img.push_back( cv::Point2f(kp2[goodMatches[i].trainIdx].pt));

    }

    double camera_matrix_data[3][3] = {
        {fx, 0, cx},
        {0, fy, cy},
        {0, 0, 1}
    };

    // 构建相机矩阵
    cv::Mat cameraMatrix( 3, 3, CV_64F, camera_matrix_data );
    cv::Mat rvec, tvec, inliers;
    // 求解pnp cv::solvePnPRansac( pts_obj, pts_img, cameraMatrix, cv::Mat(), rvec, tvec, false, 100, 1.0, 0.99, inliers );
    cv::solvePnPRansac( pts_obj, pts_img, cameraMatrix, cv::Mat(), rvec, tvec, false, 100, 1.0, 0.99, inliers );

    cout<<"inliers: "<<inliers.rows<<endl;
    cout<<"R="<<rvec<<endl;
    cout<<"t="<<tvec<<endl;

    // 画出inliers匹配 
    vector< cv::DMatch > matchesShow;
    for (size_t i=0; i<inliers.rows; i++){
        matchesShow.push_back( goodMatches[inliers.ptr<int>(i)[0]] );    
    }
    cv::drawMatches( rgb1, kp1, rgb2, kp2, matchesShow, imgMatches );
    cv::imshow( "inlier matches", imgMatches );
    cv::waitKey( 0 );
    return 0;
}