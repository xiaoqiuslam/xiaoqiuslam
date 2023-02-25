#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

/**
 * @brief 像素坐标转相机坐标

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
Point2f pixel2cam(const Point2d &p, const Mat &K) {
    return Point2f
            (
                    (p.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
                    (p.y - K.at<double>(1, 2)) / K.at<double>(1, 1)
            );
}


void find_feature_matches(
        const Mat &img_1,
        const Mat &img_2,
        std::vector<KeyPoint> &keypoints_1,
        std::vector<KeyPoint> &keypoints_2,
        std::vector<DMatch> &matches) {
    //-- 初始化
    Mat descriptors_1, descriptors_2;
    Ptr<FeatureDetector> detector = ORB::create();
    Ptr<DescriptorExtractor> descriptor = ORB::create();
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");
    //-- 第一步:检测 Oriented FAST 角点位置
    detector->detect(img_1, keypoints_1);
    detector->detect(img_2, keypoints_2);

    //-- 第二步:根据角点位置计算 BRIEF 描述子
    descriptor->compute(img_1, keypoints_1, descriptors_1);
    descriptor->compute(img_2, keypoints_2, descriptors_2);

    //-- 第三步:对两幅图像中的BRIEF描述子进行匹配，使用 Hamming 距离
    vector<DMatch> match;
    // BFMatcher matcher ( NORM_HAMMING );
    matcher->match(descriptors_1, descriptors_2, match);

    //-- 第四步:匹配点对筛选
    double min_dist = 10000, max_dist = 0;

    //找出所有匹配之间的最小距离和最大距离, 即是最相似的和最不相似的两组点之间的距离
    for (int i = 0; i < descriptors_1.rows; i++) {
        double dist = match[i].distance;
        if (dist < min_dist) min_dist = dist;
        if (dist > max_dist) max_dist = dist;
    }

    printf("-- Max dist : %f \n", max_dist);
    printf("-- Min dist : %f \n", min_dist);

    //当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限.
    for (int i = 0; i < descriptors_1.rows; i++) {
        if (match[i].distance <= max(2 * min_dist, 30.0)) {
            matches.push_back(match[i]);
        }
    }
}


void pose_estimation_2d2d(
        const std::vector<KeyPoint> &keypoints_1,
        const std::vector<KeyPoint> &keypoints_2,
        const std::vector<DMatch> &matches,
        Mat &R, Mat &t) {
    // 相机内参,TUM Freiburg2
    Mat K = (Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);

    //-- 把匹配点转换为vector<Point2f>的形式
    vector<Point2f> points1;
    vector<Point2f> points2;

    for (int i = 0; i < (int) matches.size(); i++) {
        points1.push_back(keypoints_1[matches[i].queryIdx].pt);
        points2.push_back(keypoints_2[matches[i].trainIdx].pt);
    }

    //-- 计算本质矩阵
    Point2d principal_point(325.1, 249.7);        //相机光心, TUM dataset标定值
    int focal_length = 521;            //相机焦距, TUM dataset标定值
    Mat essential_matrix;
    essential_matrix = findEssentialMat(points1, points2, focal_length, principal_point);

    //-- 从本质矩阵中恢复旋转和平移信息.
    recoverPose(essential_matrix, points1, points2, R, t, focal_length, principal_point);
}

/**
 *
 * 组装R和t成为T2
  R [0.9985534106102478,   -0.05339308467584758, 0.006345444621108698;
     0.05321959721496264,   0.9982715997131746,  0.02492965459802003;
    -0.007665548311697523, -0.02455588961730239, 0.9996690690694516]

  t[-0.8829934995085544;
    -0.05539655431450562;
     0.4661048182498402]

   T2[0.9985534,    -0.053393085, 0.0063454448, -0.88299352;
      0.053219598,   0.99827158,  0.024929654,  -0.055396553;
     -0.0076655485, -0.02455589,  0.99966908,    0.46610481]

三角化算出来的是匹配的特征点的对应的3D空间点的坐标
cv::triangulatePoints(T1, T2, pts_1, pts_2, pts_4d);
cv::triangulatePoints函数接受的参数是两个相机位姿和特征点在两个相机坐标系下的坐标，
输出三角化后的特征点的3D坐标。但需要注意的是，输出的3D坐标是齐次坐标，共四个维度，
因此需要将前三个维度除以第四个维度以得到非齐次坐标xyz。
这个坐标是在相机坐标系下的坐标，以输入的两个相机位姿所在的坐标系为准。
在主函数中，通过把3D坐标重投影到两个相机的归一化平面上，从而计算重投影误差。
因此需要再次对xyz坐标同时除以z，以得到归一化平面上的坐标。
计算点的三维坐标，与函数 reprojectImageTo3D() 不同，这里不需要视差。
 triangulatePoints（）仅需要内参数和外参数，将匹配点的像素坐标转换到相机坐标（projPoints1 ，projPoints2）。
 */
void triangulation(const vector<KeyPoint> &keypoint_1,const vector<KeyPoint> &keypoint_2,const std::vector<DMatch> &matches,const Mat &R, const Mat &t,vector<Point3d> &points) {
    // 这里是将第一个相机平面的坐标点转化到第二个相机平面的坐标点,变换矩阵是4＊4的矩阵
    Mat T1 = (Mat_<float>(3, 4) <<1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0);
    Mat T2 = (Mat_<float>(3, 4) <<R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), t.at<double>(0, 0),R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), t.at<double>(1, 0),R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), t.at<double>(2, 0));
    // 相机内参数
    Mat K = (Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
    vector<Point2f> pts_1, pts_2;
    for (DMatch m:matches) {
        pts_1.push_back(pixel2cam(keypoint_1[m.queryIdx].pt, K));
        pts_2.push_back(pixel2cam(keypoint_2[m.trainIdx].pt, K));
    }
    Mat pts_4d;
    // 三角化算出来的是匹配的特征点的对应的的3D空间点的坐标
    // pts_1, pts_2 是又特征点就是像素点转化到相机坐标系下面的点
    cv::triangulatePoints(T1, T2, pts_1, pts_2, pts_4d);
    cout << "pts_4d.size" << pts_4d.size << endl;
    cout << "pts_4d.size" << pts_4d.data << endl;

    // pts_4d.size　4 x 81 　
    // 81组匹配点　对应的81个3D点

    // 转换成非齐次坐标
    cout << pts_4d.cols << endl;
    // 81
    for (int i = 0; i < pts_4d.cols; i++) {
        Mat x = pts_4d.col(i);
        cout << "x" << x << endl;
//      [ 0.012742957;
//        0.28752118;
//       -0.95757967;
//       -0.014504699]

// 获取　第四个数字　进行归一化
        x /= x.at<float>(3, 0); // 归一化

        cout << "x" << x << endl;
//      x[-0.87853992;
//       -19.822624;
//        66.018585;
//         1]

        Point3d p(
                x.at<float>(0, 0),
                x.at<float>(1, 0),
                x.at<float>(2, 0)
        );
        points.push_back(p);
    }
}


inline cv::Scalar get_color(float depth) {
    float up_th = 50, low_th = 10, th_range = up_th - low_th;
    if (depth > up_th) depth = up_th;
    if (depth < low_th) depth = low_th;
    return cv::Scalar(255 * depth / th_range, 0, 255 * (1 - depth / th_range), 10);
}


int main(int argc, char **argv) {

    Mat img_1 = imread("../rgb1.png", cv::IMREAD_COLOR);
    Mat img_2 = imread("../rgb2.png", cv::IMREAD_COLOR);

    vector<KeyPoint> keypoints_1, keypoints_2;
    vector<DMatch> matches;
    find_feature_matches(img_1, img_2, keypoints_1, keypoints_2, matches);
    cout << "一共找到了" << matches.size() << "组匹配点" << endl;

    //-- 估计两张图像间运动
    Mat R, t;
    pose_estimation_2d2d(keypoints_1, keypoints_2, matches, R, t);
    cout << "R" << R;
    cout << "T" << t;

    //-- 单目三角化
    vector<Point3d> points;
    triangulation(keypoints_1, keypoints_2, matches, R, t, points);

    //-- 验证 三角化点 与特征点 的重投影关系
    // 相机内参数　
    Mat K = (Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
    Mat img1_plot = img_1.clone();
    Mat img2_plot = img_2.clone();
    for (int i = 0; i < matches.size(); i++) {
        // 第一个图
        float depth1 = points[i].z;
        cout << "depth: " << depth1 << endl;
        Point2d pt1_cam = pixel2cam(keypoints_1[matches[i].queryIdx].pt, K);
        // get_color(depth1) 函数根据3D点的深度信息,在图像上显示出相同的深度颜色相同的小圆圈
        cv::circle(img1_plot, keypoints_1[matches[i].queryIdx].pt, 2, get_color(depth1), 2);

        // 第二个图　在第二个相机坐标系上3D点(路标点)的三维坐标值
        Mat pt2_trans = R * (Mat_<double>(3, 1) << points[i].x, points[i].y, points[i].z) + t;
        float depth2 = pt2_trans.at<double>(2, 0);
        cv::circle(img2_plot, keypoints_2[matches[i].trainIdx].pt, 4, get_color(depth2), 2);
        // 如果将两张图片拼成一张图片，然后分别显示深度相同发的圆圈是否可行呢？
        // 还有单目的两帧图像，和双目的左右图像有什么区别呢？
    }
    cv::imshow("img 1", img1_plot);
    cv::imshow("img 2", img2_plot);
    cv::waitKey();

    return 0;
}


