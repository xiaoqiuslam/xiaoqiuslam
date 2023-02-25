


// 矩形绘制：rectangle()
// CV_IN_OUT Mat& img, // 输出图像
// Rect rec, //矩形的位置和长宽
// const Scalar& color, // 矩形颜色
// int thickness = 1, //线宽
// int lineType = LINE_8, //直线类型
// int shift = 0 //点坐标的小数点位数
cv::Rect rect = cv::Rect(300, 300, 200, 200);
cv::Scalar color = cv::Scalar(255, 0, 0);
cv::rectangle(image, rect, color, 2, 8);
imshow("矩形绘制", image);
cv::waitKey(0);
