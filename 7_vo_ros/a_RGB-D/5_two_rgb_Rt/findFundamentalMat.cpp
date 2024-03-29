#include <iostream>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/opencv.hpp>

int main(){

    cv::Mat rgb1 = cv::imread( "../rgb1.png");
    cv::Mat rgb2 = cv::imread( "../rgb2.png");

    cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
    cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();

    std::vector< cv::KeyPoint > kp1, kp2;
    detector->detect( rgb1, kp1 );
    detector->detect( rgb2, kp2 );


    cv::Mat desp1, desp2;
    descriptor->compute( rgb1, kp1, desp1 );
    descriptor->compute( rgb2, kp2, desp2 );

    std::vector< cv::DMatch > matches;
    cv::BFMatcher matcher;
    matcher.match( desp1, desp2, matches );
    std::cout<<"Find total "<<matches.size()<<" matches."<< std::endl;


    std::vector< cv::DMatch > goodMatches;
    double minDis = 9999;
    for ( size_t i=0; i<matches.size(); i++ )
    {
        if ( matches[i].distance < minDis )
            minDis = matches[i].distance;
    }

    for ( size_t i=0; i<matches.size(); i++ )
    {
        if (matches[i].distance < 10*minDis)
            goodMatches.push_back( matches[i] );
    }


    std::vector< cv::Point2f > pts1, pts2;
    for (size_t i=0; i<goodMatches.size(); i++)
    {
        pts1.push_back(kp1[goodMatches[i].queryIdx].pt);
        pts2.push_back(kp2[goodMatches[i].trainIdx].pt);
    }

    // 计算基础矩阵并据此绘制出前10个匹配点对应的对极线
    cv::Mat F = findFundamentalMat(pts1, pts2, cv::FM_8POINT);
    std::vector<cv::Vec<float, 3>> epilines1, epilines2;
    //计算对应点的外极线epilines是一个三元组(a,b,c)，表示点在另一视图中对应的外极线ax+by+c=0;
    computeCorrespondEpilines(pts1, 1, F, epilines1);
    computeCorrespondEpilines(pts2, 2, F, epilines2);

    cv::RNG rng;
    for (uint i = 0; i < 10; i++){
        cv::Scalar color = cv::Scalar(rng(256), rng(256), rng(256));//随机产生颜色

        //在视图2中把关键点用圆圈画出来，然后再绘制在对应点处的外极线
        cv::circle(rgb2, pts2[i], 3, color, 2);
        line(rgb2, cv::Point(0, -epilines1[i][2] / epilines1[i][1]), cv::Point(rgb2.cols, -(epilines1[i][2] + epilines1[i][0] * rgb2.cols) / epilines1[i][1]), color);

        //绘制外极线的时候，选择两个点，一个是x=0处的点，一个是x为图片宽度处
        circle(rgb1, pts1[i], 3, color, 2);
        line(rgb1, cv::Point(0, -epilines2[i][2] / epilines2[i][1]), cv::Point(rgb1.cols, -(epilines2[i][2] + epilines2[i][0] * rgb1.cols) / epilines2[i][1]), color);

    }
    imshow("epiline1", rgb2);
    imshow("epiline2", rgb1);
    cv::waitKey(0);
    return 0;
}
