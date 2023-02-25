
//设置绘制文本的相关参数
// 待绘制的文字
std::string text = "SLAM";
// 字体 (如cv::FONT_HERSHEY_PLAIN)
int font_face = cv::FONT_HERSHEY_TRIPLEX;
// 尺寸因子，值越大文字越大
double font_scale = 2;
// 线条宽度
int thickness = 2;
int baseline;
//获取文本框的长宽
cv::Size text_size = cv::getTextSize(text, font_face, font_scale, thickness, &baseline);

//将文本框居中绘制  文本框的左下角
cv::Point origin;
origin.x = image.cols / 2 - text_size.width / 2;
origin.y = image.rows / 2 + text_size.height / 2;
//  cv::Mat& image,  待绘制的图像
//  cv::Scalar color, // 线条的颜色（RGB）
//  int lineType = 8, // 线型（4邻域或8邻域，默认8邻域）
// bool bottomLeftOrigin = false // true='origin at lower left'
cv::putText(image, text, origin, font_face, font_scale, cv::Scalar(0, 0, 255), thickness, 8, 0);

//显示绘制解果
cv::imshow("图片上绘制文字", image);
cv::waitKey(0);