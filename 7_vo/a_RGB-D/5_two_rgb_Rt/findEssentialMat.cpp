#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

void find_feature_matches(const cv::Mat &img_1, const cv::Mat &img_2, std::vector<cv::KeyPoint> &keypoints_1, std::vector<cv::KeyPoint> &keypoints_2, std::vector<cv::DMatch> &matches) {

    // 第一步:提取特征点
    cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
    detector->detect(img_1, keypoints_1);
    detector->detect(img_2, keypoints_2);

    // 第二步:计算 特征点 BRIEF 描述子
    cv::Mat descriptors_1, descriptors_2;
    cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();
    descriptor->compute(img_1, keypoints_1, descriptors_1);
    descriptor->compute(img_2, keypoints_2, descriptors_2);

    // 第三步:对两幅图像中的BRIEF描述子进行匹配，使用 Hamming 距离
    std::vector<cv::DMatch> match;
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
    matcher->match(descriptors_1, descriptors_2, match);

    // 第四步:匹配点对筛选
    double min_dist = 10000, max_dist = 0;

    // 找出所有匹配之间的最小距离和最大距离, 即是最相似的和最不相似的两组点之间的距离
    for (int i = 0; i < descriptors_1.rows; i++) {
        double dist = match[i].distance;
        if (dist < min_dist) min_dist = dist;
        if (dist > max_dist) max_dist = dist;
    }

    // 当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限.
    for (int i = 0; i < descriptors_1.rows; i++) {
        if (match[i].distance <= cv::max(2 * min_dist, 30.0)) {
            matches.push_back(match[i]);
        }
    }
}


void pose_estimation_2d2d(std::vector<cv::KeyPoint> keypoints_1, std::vector<cv::KeyPoint> keypoints_2, std::vector<cv::DMatch> matches, cv::Mat &R, cv::Mat &t) {
    // 相机内参,TUM Freiburg2
    cv::Mat K = (cv::Mat_<double>(3, 3) <<
                                520.9, 0,     325.1,
                                0,     521.0, 249.7,
                                0,     0,     1);

    // 特征点转换为vector<Point2f>的形式
    std::vector<cv::Point2f> points1;
    std::vector<cv::Point2f> points2;

    // 匹配特征点matches的索引,在特征点keypoints_1中搜索,将匹配的特征点存储到points1,points2
    for (int i = 0; i < (int) matches.size(); i++) {
        points1.push_back(keypoints_1[matches[i].queryIdx].pt);
        points2.push_back(keypoints_2[matches[i].trainIdx].pt);
    }

    //-- 计算基础矩阵
    cv::Mat fundamental_matrix;
    fundamental_matrix = findFundamentalMat ( points1, points2, cv::FM_8POINT );
    std::cout<<"fundamental_matrix is "<<std::endl<< fundamental_matrix<<std::endl;

    //-- 计算单应矩阵
    cv::Mat homography_matrix;
    homography_matrix = findHomography ( points1, points2, cv::RANSAC, 3 );
    std::cout<<"homography_matrix is "<<std::endl<<homography_matrix<<std::endl;

    // 计算本质矩阵
    // TUM dataset 相机焦距
    double focal_length = 521;
    // 相机光心, TUM dataset 标定值
    cv::Point2d principal_point(325.1, 249.7);
    // 计算本质矩阵
    cv::Mat essential_matrix = findEssentialMat(points1, points2, focal_length, principal_point);
    std::cout << "essential_matrix is " << std::endl << essential_matrix << std::endl;

    // 从本质矩阵中恢复旋转R和平移t信息
    recoverPose(essential_matrix, points1, points2, R, t, focal_length, principal_point);
}


/**
 * @brief 像素坐标转相机归一化坐标
 *
 * 相机内参
 * fx     0      cx
 * 0      fy     cy
 * 0      0      1
 *
 * 520.9, 0,     325.1,
 * 0,     521.0, 249.7,
 * 0,     0,     1
 *
 * K.at<double>(0, 2) 325.1  cx
 * K.at<double>(0, 0) 520.9  fx
 * K.at<double>(1, 2) 249.7  cy
 * K.at<double>(1, 1) 521    fy
 */
cv::Point2d pixel2cam(const cv::Point2d &p, const cv::Mat &K) {
    return cv::Point2d(
            (p.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
            (p.y - K.at<double>(1, 2)) / K.at<double>(1, 1)
    );
}


int main(int argc, char **argv) {
    cv::Mat img_1 = imread("../rgb1.png", cv::IMREAD_COLOR);
    cv::Mat img_2 = imread("../rgb2.png", cv::IMREAD_COLOR);

    // 计算两张图像之间匹配的特征点
    std::vector<cv::KeyPoint> keypoints_1, keypoints_2;
    std::vector<cv::DMatch> matches;
    find_feature_matches(img_1, img_2, keypoints_1, keypoints_2, matches);
    std::cout << "一共找到了" << matches.size() << "组匹配点" << std::endl;

    // 根据两张图像之间匹配的特征点计算两张图像之间的变换矩阵R,t
    cv::Mat R, t;
    pose_estimation_2d2d(keypoints_1, keypoints_2, matches, R, t);
    std::cout << "R is " << std::endl << R << std::endl;
    std::cout << "t is " << std::endl << t << std::endl;

    // E = t^R*scale 十四讲公式 7.10 第一个
    cv::Mat t_x =(cv::Mat_<double>(3, 3) <<0,                       -t.at<double>(2, 0), t.at<double>(1, 0),
                                                    t.at<double>(2, 0),  0,                      -t.at<double>(0, 0),
                                                   -t.at<double>(1, 0),  t.at<double>(0, 0), 0);

    std::cout << "t_x: " << std::endl << t_x << std::endl;
    std::cout << "t^R=" << std::endl << t_x * R << std::endl;


    cv::Mat K = (cv::Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);

    for (cv::DMatch m: matches) {
        // 像素点转换为成像平面的点，再转换为归一化平面的点
        cv::Point2d pt1 = pixel2cam(keypoints_1[m.queryIdx].pt, K);
        cv::Mat y1 = (cv::Mat_<double>(3, 1) << pt1.x, pt1.y, 1); // 归一化平面上面的点

        cv::Point2d pt2 = pixel2cam(keypoints_2[m.trainIdx].pt, K);
        cv::Mat y2 = (cv::Mat_<double>(3, 1) << pt2.x, pt2.y, 1); // 归一化平面上面的点

        // 十四讲公式 7.10 第三个等号左侧
        cv::Mat d = y2.t() * t_x * R * y1;
        std::cout << "epipolar constraint = " << d << std::endl;
    }
    return 0;
}
