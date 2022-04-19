// 两张图像求单应矩阵求Rt，然后用矩阵将一张图像变换，完后选择特定区域将变换后的图像叠加到另一张图像上

#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

int main(){
    Mat im_src = imread("../bot.png");
    Size size = im_src.size();

    vector<Point2f> pts_src;
    pts_src.push_back(Point2f(0,0));
    pts_src.push_back(Point2f(size.width - 1, 0));
    pts_src.push_back(Point2f(size.width - 1, size.height -1));
    pts_src.push_back(Point2f(0, size.height - 1 ));
    
    Mat im_dst = imread("../ad.jpg");
    Mat im_temp = im_dst.clone();
    vector<Point2f> pts_dst1;
    pts_dst1.push_back(Point2f(450,74));
    pts_dst1.push_back(Point2f(799, 377));
    pts_dst1.push_back(Point2f(809, 516));
    pts_dst1.push_back(Point2f(450, 391));

    // 计算原图四个点和目标图区域四个点的 Homography 
    Mat Homography = findHomography(pts_src, pts_dst1);

    // 用H对原图做变换
    warpPerspective(im_src, im_temp, Homography, im_temp.size());

    // 四个点
    Point pts_dst[4];
    for( int i = 0; i < 4; i++){
        pts_dst[i] = pts_dst1[i];
    }
    // 函数 fillConvexPoly 把目标图中对应区域像素值设置为0
    fillConvexPoly(im_dst, pts_dst, 4, Scalar(0), CV_AA);
    // 把原图叠加到目标图上
    im_dst = im_dst + im_temp;
    imshow("im_dst", im_dst);
    waitKey(0);
    return 0;
}
