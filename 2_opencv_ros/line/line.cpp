// 红色图像上 画上蓝色的线
// 在图中画一条直线 函数原型
// 函数作用
// 给定一个图像img，连接点pt1和pt2的坐标，在图中画一条直线，color表明线的颜色cv.line()
// 其中需要注意的是，点坐标(x, y)中，x代表图片的列，y代表图片的行
// CvPoint pt1, //直线起点
// CvPoint pt2, //直线终点
// CvScalar color, //直线的颜色 Scalor(0,0,255)
// int thickness=1, //线条粗细
// int line_type=8, //8-connected line（8邻接)连接线。
// //4-connected line(4邻接)连接线。
// //CV_AA - antialiased 线条。
// int shift=0 //坐标点的小数点位数。
cv::line(img_red, cv::Point(0, 50), cv::Point(img_red.cols, 50), cv::Scalar(0,255,0,0), 10);
cv::line(img_red, cv::Point(0, 100), cv::Point(img_red.cols, 100), cv::Scalar(0,255,0,255), 10);

cv::imshow("img_blue", img_red);
// 暂停程序,等待一个按键输入,如果不暂停画面会一闪而过
cv::waitKey(0);