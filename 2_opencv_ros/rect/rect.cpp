/**
 * 图像剪切操作
 * Rect矩形类包括Point点类的成员x和y(表示矩形的左上角)以及size类的成员width和height(表示矩形的大小)。
 * 可以设置图像ROI区域，截取图像等。
 * 成员变量有x,y,width,height，分别为左上角坐标和矩形宽高。常用函数有Size()返回Size;
 * area()返回矩形面积;
 * contains(Point)判断点是否在矩形内;
 * inside(Rect)判断矩形是否在该矩形内;
 * tl()返回左上角点座标;
 * br()返回右下角点座标;
 */
cv::Rect rect1(300, 300, 200, 200);
cv::Rect rect2(200, 200, 200, 200);
cv::Mat roi1;
image(rect1).copyTo(roi1); // copy the region rect1 from the image to roi1
imshow("roi1", roi1);
cv::waitKey(0);

cv::Mat roi2;
image(rect2).copyTo(roi2); // copy the region rect2 from the image to roi2
imshow("roi2", roi2);
cv::waitKey(0);

// 求两个矩形的交集 就是两个区域公共的部分
cv::Rect rect3 = rect1&rect2;
cv::Mat roi3;
image(rect3).copyTo(roi3);
imshow("交&", roi3);
cv::waitKey(0);

// 求两个矩形的并集 两个区域加到一起
cv::Rect rect4 = rect1|rect2;
cv::Mat roi4;
image(rect4).copyTo(roi4);
imshow("并|", roi4);
cv::waitKey(0);




// 把第一个区域图区出来粘贴到 image 上面
cv::Rect rect5(10, 10, 200, 200);
roi1.copyTo(image(rect5)); // copy the region rect1 to the designated region in the image
std::cout << "rect5" << std::endl ;
imshow("5", image);
cv::waitKey(0);


// 关于 cv::Mat 的拷贝
// 直接赋值并不会拷贝数据
cv::Mat image_another = image;
// 修改 image_another 会导致 image 发生变化
// 将左上角100*100的块置零
image_another(cv::Rect(0, 0, 100, 100)).setTo(0);
cv::imshow("image", image);
cv::waitKey(0);

// 使用clone函数来拷贝数据
cv::Mat image_clone = image.clone();
image_clone(cv::Rect(0, 0, 100, 100)).setTo(255);
cv::imshow("image", image);
cv::imshow("image_clone", image_clone);
cv::waitKey(0);