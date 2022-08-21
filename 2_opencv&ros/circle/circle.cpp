/**
 * void cvCircle( CvArr* img, CvPoint center, int radius, CvScalar color, int thickness=1, int line_type=8, int shift=0 );
 * Opencv画点 其实画的是小圆圈
 * img：图像。
 * center：圆心坐标。
 * radius：圆形的半径。
 * color：线条的颜色。
 * thickness：如果是正数，表示组成圆的线条的粗细程度。否则，表示圆是否被填充。
 * line_type：线条的类型。见 cvLine 的描述
 * shift：圆心坐标点和半径值的小数点位数。
 * 画圆画点都是使用circle()函数来画，点就是圆，我们平常所说的圆只不过是半径大一点而已。
 */

// 画绿色的空心点
cv::Point p(100, 100);//初始化点坐标为(50,50)
// 第三个参数表示点的半径，第四个参数选择颜色。这样子我们就画出了绿色的空心点
circle(image, p, 50, cv::Scalar(0, 255, 0),3);//第五个参数我们调高点，让线更粗


// 这种初始化点的方式也可以
cv::Point p2;
p2.x = 200;
p2.y = 200;
// 画蓝色实心点 就是圆
circle(image, p2, 50, cv::Scalar(255,0,0),-1); //第五个参数我设为-1，表明这是个实点。

imshow("画空心点-圈, 和画实心点-圆", image);
cv::waitKey();