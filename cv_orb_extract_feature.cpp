#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

/*
 * 1. 程序实现流程：
 * 2. 首先读取图像，然后缩放
 * 3. 提取特征点，然后显示提取到的特征点，这里对比了两种提取特征点的算法
 * 4. 计算特征点的描述子，并根据距离对描述子进行筛选
 * 5. 最后显示匹配比较好的特征点对
 */

int main() {
    // 0.读取图像并缩小0.5倍
    cv::Mat image_1 = cv::imread("../3.jpeg", CV_LOAD_IMAGE_COLOR);
    cv::resize(image_1, image_1, cv::Size(0, 0), 0.5, 0.5, CV_INTER_LINEAR);
    cv::Mat image_2 = cv::imread("../4.jpeg", CV_LOAD_IMAGE_COLOR);
    cv::resize(image_2, image_2, cv::Size(0, 0), 0.5, 0.5, CV_INTER_LINEAR);
    cv::imshow("image_1", image_1);
    cv::imshow("image_2", image_2);
    // cv::waitKey(0);

    std::vector<cv::KeyPoint> key_points_cv_fast_1, key_points_cv_fast_2;
    /**
     * 1.检测cv::FAST特征点,阈值设置为threshold=40
     * image_1 输入:灰度图像
     * keypoints 输出:检测到的特征点
     * threshold 中心像素的像素值和该像素周围的像素值之差的阈值
     * nonmaxSuppression 是否对特征点采用极大值抑制
     * type 像素邻域圆的三种类型：FastFeatureDetector::TYPE_9_16,FastFeatureDetector::TYPE_7_12, FastFeatureDetector::TYPE_5_8
     */
    cv::FAST(image_1, key_points_cv_fast_1, 40);
    cv::FAST(image_2, key_points_cv_fast_2, 40);
    cv::Mat key_points_cv_fast_1_show;
    cv::Mat key_points_cv_fast_2_show;
    /**
     * 2.显示检测到的关键点
     * 参数说明 绘制原始图像并把检测到的特征点画在图像上
     * img1,img2,keypoints1,keypoints2:给出了两张图片和分别的关键点
     * matches1to2:关键点的匹配关系其中 keypoints1[i] 与 keypoints2[matches[i]] 相匹配
     * outImg:匹配结果
     * flags::cv::DrawMatchesFlags::DEFAULT输出结果在outImg中同时用小圆圈标记
     * cv::DrawMatchesFlags::DRAW_OVER_OUTIMG并不重新分配outImg的空间这样可以多次调用 cv::drawMatches()将结果绘制在一张图上
     * cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS并不绘制未匹配上的关键点
     * cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS将关键点用带尺度和方向的圆标示
     */
    cv::drawKeypoints(image_1, key_points_cv_fast_1, key_points_cv_fast_1_show, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
    cv::drawKeypoints(image_2, key_points_cv_fast_2, key_points_cv_fast_2_show, cv::Scalar::all(-1),cv::DrawMatchesFlags::DEFAULT);
    cv::Mat all_image_show_fast(image_1.size() * 2, image_1.type());
    image_1.copyTo(all_image_show_fast(cv::Rect(0, 0, all_image_show_fast.cols / 2, all_image_show_fast.rows / 2)));
    image_2.copyTo(all_image_show_fast(cv::Rect(0, image_1.rows, all_image_show_fast.cols / 2, all_image_show_fast.rows / 2)));
    key_points_cv_fast_1_show.copyTo(all_image_show_fast(cv::Rect(image_1.cols, 0, all_image_show_fast.cols / 2, all_image_show_fast.rows / 2)));
    key_points_cv_fast_2_show.copyTo(all_image_show_fast(cv::Rect(image_1.cols, image_1.rows, all_image_show_fast.cols / 2, all_image_show_fast.rows / 2)));
    cv::imshow("all_image_show_fast", all_image_show_fast);
    // cv::waitKey(0);

    // 3.检测 Oriented FAST 特征点
    std::vector<cv::KeyPoint> key_points_cv_orb_fast_1, key_points_cv_orb_fast_2;
    // 创建特征点提取器(sift, surf, ORB)
    cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
    detector->detect(image_1, key_points_cv_orb_fast_1);
    detector->detect(image_2, key_points_cv_orb_fast_2);
    cv::Mat key_points_cv_orb_fast_1_show;
    cv::drawKeypoints(image_1, key_points_cv_orb_fast_1, key_points_cv_orb_fast_1_show, cv::Scalar::all(-1),cv::DrawMatchesFlags::DEFAULT);
    // Scalar::all(-1)随机颜色 cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS 绘制每个关键的的大小和方向
    cv::Mat key_points_cv_orb_fast_2_show;
    // 4.显示检测到的关键点
    cv::drawKeypoints(image_2, key_points_cv_orb_fast_2, key_points_cv_orb_fast_2_show, cv::Scalar::all(-1),cv::DrawMatchesFlags::DEFAULT);
    cv::Mat all_image_show_orb_fast(image_1.size() * 2, image_1.type());
    image_1.copyTo(all_image_show_orb_fast(cv::Rect(0, 0, all_image_show_orb_fast.cols / 2, all_image_show_orb_fast.rows / 2)));
    image_2.copyTo(all_image_show_orb_fast(cv::Rect(0, image_1.rows, all_image_show_orb_fast.cols / 2, all_image_show_orb_fast.rows / 2)));
    key_points_cv_orb_fast_1_show.copyTo(all_image_show_orb_fast(cv::Rect(image_1.cols, 0,all_image_show_orb_fast.cols / 2,all_image_show_orb_fast.rows / 2)));
    key_points_cv_orb_fast_2_show.copyTo(all_image_show_orb_fast(cv::Rect(image_1.cols, image_1.rows,all_image_show_orb_fast.cols / 2,all_image_show_orb_fast.rows / 2)));
    cv::imshow("all_image_show_orb_fast", all_image_show_orb_fast);
    // cv::waitKey(0);

    // 5.计算特征点的BRIEF描述子
    cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();
    cv::Mat descriptors_1, descriptors_2;
    descriptor->compute(image_1, key_points_cv_orb_fast_1, descriptors_1);
    descriptor->compute(image_2, key_points_cv_orb_fast_2, descriptors_2);
    //对两幅图像中的BRIEF描述子使用Hamming距离进行匹配
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
    std::vector<cv::DMatch> matches;
    matcher->match(descriptors_1, descriptors_2, matches);
    cv::Mat img_match;
    // 6.显示所有匹配的特征点对
    drawMatches(image_1, key_points_cv_orb_fast_1, image_2, key_points_cv_orb_fast_2, matches, img_match);
    imshow("all match point", img_match);
    // cv::waitKey(0);

    // 7.筛选匹配的特征点对
    double min_dist = 10000, max_dist = 0;
    for (int i = 0; i < descriptors_1.rows; i++) {
        // 计算匹配点的vector中匹配的特征点对的最大的距离和最小的距离,过滤掉很小的的距离,很小的距离代表两个特征点很相似,会出现特征点扎堆现象
        double dist = matches[i].distance;
        if (dist < min_dist)
            min_dist = dist;
        if (dist > max_dist)
            max_dist = dist;
    }
    std::vector<cv::DMatch> good_matches;
    for (int i = 0; i < descriptors_1.rows; i++) {
        //当描述子之间的距离大于两倍的最小距离时,即认为匹配有误但有时候最小距离会非常小,设置一个经验值30作为下限,因为距离太远的可以认为不是匹配的特征点
        if (matches[i].distance <= cv::max(1.2 * min_dist, 40.0)) {
            good_matches.push_back(matches[i]);
        }
    }
    cv::Mat img_good_match;
    drawMatches(image_1, key_points_cv_orb_fast_1, image_2, key_points_cv_orb_fast_2, good_matches, img_good_match);
    imshow("filter match point", img_good_match);
    cv::waitKey(0);
    cv::destroyAllWindows();
    return 0;
}

