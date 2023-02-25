#include <opencv2/opencv.hpp>

// 两张图像求单应矩阵求Rt，然后用矩阵将一张图像变换，完后选择特定区域将变换后的图像叠加到另一张图像上
int main(){
    cv::Mat im_src = cv::imread("../xiaoqiu.png");
    std::vector<cv::Point2f> pts_src;
    pts_src.push_back(cv::Point2f(0,0));
    circle(im_src, cv::Point2f(0,0), 5, cv::Scalar(255,0,0),-1);
    pts_src.push_back(cv::Point2f(im_src.size().width - 1, 0));
    circle(im_src, cv::Point2f(im_src.size().width - 1, 0), 5, cv::Scalar(255,0,0),-1);
    pts_src.push_back(cv::Point2f(im_src.size().width - 1, im_src.size().height -1));
    circle(im_src, cv::Point2f(im_src.size().width - 1, im_src.size().height -1), 5, cv::Scalar(255,0,0),-1);
    pts_src.push_back(cv::Point2f(0, im_src.size().height - 1 ));
    circle(im_src, cv::Point2f(0, im_src.size().height - 1 ), 5, cv::Scalar(255,0,0),-1);
    imshow("boder", im_src);
    cv::waitKey();


    cv::Mat im_dst = cv::imread("../ad.jpg");
    std::vector<cv::Point2f> pts_dst1;
    pts_dst1.push_back(cv::Point2f(450,74));
    circle(im_dst, cv::Point2f(450,74), 5, cv::Scalar(255,0,0),-1);
    pts_dst1.push_back(cv::Point2f(799, 377));
    circle(im_dst, cv::Point2f(799, 377), 5, cv::Scalar(255,0,0),-1);
    pts_dst1.push_back(cv::Point2f(809, 516));
    circle(im_dst, cv::Point2f(809, 516), 5, cv::Scalar(255,0,0),-1);
    pts_dst1.push_back(cv::Point2f(450, 391));
    circle(im_dst, cv::Point2f(450, 391), 5, cv::Scalar(255,0,0),-1);
    imshow("mubiao", im_dst);
    cv::waitKey();

    // 计算原图四个点和目标图区域四个点的 Homography
    cv::Mat Homography = findHomography(pts_src, pts_dst1);

    // 用H对原图做变换
    cv::Mat im_temp = im_dst.clone();
    warpPerspective(im_src, im_temp, Homography, im_temp.size());
    imshow("im_temp", im_temp);
    cv::waitKey();

    // 四个点
    cv::Point pts_dst[4];
    for( int i = 0; i < 4; i++){
        pts_dst[i] = pts_dst1[i];
    }
    // 函数 fillConvexPoly 把目标图中对应区域像素值设置为0
    fillConvexPoly(im_dst, pts_dst, 4, cv::Scalar(0, 0,0), cv::LINE_AA);
    imshow("im_dst1", im_dst);
    cv::waitKey();

    // 把原图叠加到目标图上
    im_dst = im_dst + im_temp;
    imshow("im_dst", im_dst);
    cv::waitKey(0);
    return 0;
}