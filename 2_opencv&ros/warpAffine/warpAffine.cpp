#include <iostream>
#include <chrono>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

int main(int argc, char **argv) {
cv::Mat image;
image = cv::imread(argv[1]);

// 图像旋转
// 第一个参数为旋转中心，第二个为旋转角度，第三个为旋转后的缩放因子

cv::Mat rotated_img;
//float scale = 200.0/ src.rows;//缩放因子
//cv::resize(src, src, cv::Size(), scale, scale, cv::INTER_LINEAR);
//输出图像的尺寸与原图一样
cv::Size dst_sz(image.cols, image.rows);
//指定旋转中心
cv::Point2f center(static_cast<float>(image.cols / 2.), static_cast<float>(image.rows / 2.));

//获取旋转矩阵（2x3矩阵）
cv::Mat rot_mat = cv::getRotationMatrix2D(center, 30, 1.0);
//设置选择背景边界颜色
/*cv::Scalar borderColor = Scalar(0, 238, 0);*/
//cv::warpAffine(src, dst, rot_mat, src.size(), INTER_LINEAR, BORDER_CONSTANT, borderColor);
//复制边缘填充
cv::warpAffine(image, rotated_img, rot_mat, dst_sz, cv::INTER_LINEAR, cv::BORDER_REFLECT101);
cv::imshow("rotated_img", rotated_img);
// 暂停程序,等待一个按键输入,避免图像一闪而过
cv::waitKey(0);






// 放射变换 warpAffine
// 如何通俗地讲解「仿射变换」这个概念？ https://www.zhihu.com/question/20666664
// 代码参考 https://zhuanlan.zhihu.com/p/24591720
/**
 *
 void warpAffine(
 InputArray src, 输入变换前图像
    OutputArray dst, 输出变换后图像，需要初始化一个空矩阵用来保存结果，不用设定矩阵尺寸
    InputArray M, 变换矩阵，用另一个函数getAffineTransform()计算
    Size dsize, 设置输出图像大小
    int flags=INTER_LINEAR, 设置插值方式，默认方式为线性插值
    int borderMode=BORDER_CONSTANT,
    const Scalar& borderValue=Scalar())


    关于生成变换矩阵InputArray M的函数getAffineTransform()：
    Mat getAffineTransform(const Point2f* src, const Point2f* dst)
    参数const Point2f* src：原图的三个固定顶点
    参数const Point2f* dst：目标图像的三个固定顶点
    返回值：Mat型变换矩阵，可直接用于warpAffine()函数
    注意，顶点数组长度超过3个，则会自动以前3个为变换顶点；数组可用Point2f[]或Point2f*表示

    */

//读取原图
//设置空矩阵用于保存目标图像
cv::Mat warpAffine_dst;
//设置原图变换顶点
cv::Point2f warpAffine_AffinePoints0[3] = { cv::Point2f(100, 50), cv::Point2f(100, 390), cv::Point2f(600, 50) };
//设置目标图像变换顶点
cv::Point2f warpAffine_AffinePoints1[3] = { cv::Point2f(200, 100), cv::Point2f(200, 330), cv::Point2f(500, 50) };
//计算变换矩阵
cv::Mat Trans = cv::getAffineTransform(warpAffine_AffinePoints0, warpAffine_AffinePoints1);
//矩阵仿射变换
warpAffine(image, warpAffine_dst, Trans, cv::Size(image.cols, image.rows));

for (int i = 0; i < 4; i++)
{
    circle(image, warpAffine_AffinePoints0[i], 2, cv::Scalar(0, 0, 255), 2);
    circle(warpAffine_dst, warpAffine_AffinePoints1[i], 2, cv::Scalar(0, 0, 255), 2);
}


//分别显示变换先后图像进行对比
imshow("src", image);
imshow("warpAffine_dst ", warpAffine_dst);
cv::waitKey();



// 透视变换 warpPerspective
/**
 * void warpPerspective(
 * InputArray src,
 * OutputArray dst,
 * InputArray M,
 * Size dsize,
 * int flags=INTER_LINEAR,
 * int borderMode=BORDER_CONSTANT,
 * const Scalar& borderValue=Scalar())
 *
 *
 * 生成变换矩阵函数为：
 * Mat getPerspectiveTransform(const Point2f* src, const Point2f* dst)
 * 透视变换顶点为4个
 */
cv::Mat warpPerspective_dst;
cv::Point2f warpPerspective_AffinePoints0[4] = { cv::Point2f(100, 50), cv::Point2f(100, 390), cv::Point2f(600, 50), cv::Point2f(600, 390) };
cv::Point2f warpPerspective_AffinePoints1[4] = { cv::Point2f(200, 100), cv::Point2f(200, 330), cv::Point2f(500, 50), cv::Point2f(600, 390) };
cv::Mat PerspectiveTrans_Trans = getPerspectiveTransform(warpPerspective_AffinePoints0, warpPerspective_AffinePoints1);
warpPerspective(image, warpPerspective_dst, PerspectiveTrans_Trans, cv::Size(image.cols, image.rows), CV_INTER_CUBIC);
for (int i = 0; i < 4; i++)
{
    circle(image, warpPerspective_AffinePoints0[i], 2, cv::Scalar(0, 0, 255), 2);
    circle(warpPerspective_dst, warpPerspective_AffinePoints1[i], 2, cv::Scalar(0, 0, 255), 2);
}
imshow("warpPerspective_dst", warpPerspective_dst);
imshow("src", image);
cv::waitKey();


// 摄影变换 明天继续把这些变换搞完？
//





















// 对于图像还有很多基本的操作,如剪切,旋转,缩放等,限于篇幅就不一一介绍了,请参看OpenCV官方文档查询每个函数的调用方法.
cv::destroyAllWindows();


return 0;
}
