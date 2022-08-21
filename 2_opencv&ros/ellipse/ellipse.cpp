


/**
 * void ellipse(
 * InputOutputArray img, // 图像
 * Point center, // 椭圆原心
 * Size axes, // 椭圆x轴长度的一半，y轴长度的一半
 * double angle, // 椭圆旋转角度
 * double startAngle, // 起始角度
 * double endAngle, // 终止角度
 * const Scalar& color, // 椭圆颜色
 * int thickness = 1, //线宽
 * int lineType = LINE_8, //线型
 * int shift = 0); //坐标小数点位数
 */
cv::Scalar color1 = cv::Scalar(0, 0, 255);
ellipse(image, cv::Point(image.cols / 2, image.rows / 2), cv::Size(image.cols / 4, image.rows / 4), 0, 0, 360, color1, 2, 8);
imshow("椭圆绘制", image);
