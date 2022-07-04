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

    /****************************************************************************************************************************
    *  常用的Mat类矩阵的元素读取方式有：通过at方法进行读取、通过指针ptr进行读取、通过迭代器进行读取、通过矩阵元素的地址定位方式进行读取。
    *  接下来将详细的介绍这四种读取方式。
    *  遍历图像, 请注意以下遍历方式亦可使用于随机像素访问
    ***************************************************************************************************************************/
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    for (size_t y = 0; y < image.rows; y++) {
      // 用cv::Mat::ptr获得图像的行指针
      unsigned char *row_ptr = image.ptr<unsigned char>(y);  // row_ptr是第y行的头指针
      for (size_t x = 0; x < image.cols; x++) {
        // 访问位于 x,y 处的像素
        unsigned char *data_ptr = &row_ptr[x * image.channels()]; // data_ptr 指向待访问的像素数据
        // 输出该像素的每个通道,如果是灰度图就只有一个通道
        for (int c = 0; c != image.channels(); c++) {
          unsigned char data = data_ptr[c]; // data为I(x,y)第c个通道的值
        }
      }
    }
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_used = std::chrono::duration_cast < std::chrono::duration < double >> (t2 - t1);
    std::cout << "遍历图像用时：" << time_used.count() << " 秒。" << std::endl;




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

    // 图像添加噪声
    cv::Mat dstImage = image.clone();
    for (int k = 0; k < 3000; k++){
        //随机取值行列
        int i = rand() % dstImage.rows;
        int j = rand() % dstImage.cols;
        //图像通道判定
        // 如果是单通道 直接把像素变成白色
        if (dstImage.channels() == 1){
            dstImage.at<uchar>(i, j) = 255;		//盐噪声
            // 行()索引  列[]索引
        }
        else{
            // 蓝色像素
            // 行()索引  列[]索引
            dstImage.at<cv::Vec3b>(i, j)[0] = 255;
            dstImage.at<cv::Vec3b>(i, j)[1] = 0;
            dstImage.at<cv::Vec3b>(i, j)[2] = 0;
        }
    }
    cv::imshow("dstImage", dstImage);
    // 暂停程序,等待一个按键输入,避免图像一闪而过
    cv::waitKey(0);


    // 高斯模糊
    cv::Mat GaussianBlur_img;
    // 标准差参数设置为0是指根据窗口大小（5,5）来自行计算高斯函数标准差
    cv::GaussianBlur(image, GaussianBlur_img, cv::Size(5, 5), 3, 3,0);

    cv::imshow("GaussianBlur_img", GaussianBlur_img);
    // 暂停程序,等待一个按键输入,避免图像一闪而过
    cv::waitKey(0);

    // 双边滤波
    cv::Mat bilateralFilter_img;
    // 9 代表邻域直径，两个参数75分别代表值域与空域标准差
    cv::bilateralFilter(image, bilateralFilter_img, 9, 75, 75);
    cv::imshow("bilateralFilter_img", bilateralFilter_img);
    cv::waitKey(0);


    // 中值滤波
    cv::Mat medianBlur_img;
    // 参数5表示选择附近5*5区域的像素值进行计算
    cv::medianBlur(image, medianBlur_img, 5);
    cv::imshow("bilateralFilter_img", medianBlur_img);
    cv::waitKey(0);


    // 转换为灰度图像
    cv::Mat gray_img;
    cv::cvtColor(image, gray_img, cv::COLOR_BGR2GRAY);
    std::cout << "gray_img.size" << gray_img.size << std::endl;
    std::cout << "image.channels()\n" << image.channels() << std::endl;
    std::cout << "gray_img.channels()\n" << gray_img.channels() << std::endl;
    cv::imshow("gray_img", gray_img);
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



    // resize缩放
    /**
     * resize(
     * InputArray src,  输入待改变大小的原图像
     * OutputArray dst, 输出改变后的图像,图像和原图像具有相同的内容，只是大小和原图像不一样而已
     * Size dsize, 输出图像的大小。
     * 如果这个参数不为0，那么就代表将原图像缩放到这个Size(width，height)指定的大小；
     * 如果这个参数为0，那么原图像缩放之后的大小就要通过下面的公式来计算：
     * 貌似图像金字塔中用到的是图像的比例
     * dsize = Size(round(fxsrc.cols), round(fysrc.rows))
     * 其中，fx和fy就是下面要说的两个参数，是图像width方向和height方向的缩放比例。
     * fx：width方向的缩放比例，如果它是0，那么它就会按照(double)dsize.width/src.cols来计算；
     * fy：height方向的缩放比例，如果它是0，那么它就会按照(double)dsize.height src.rows来计算；
     * double fx=0,
     * double fy=0,
     *
     * int interpolation=INTER_LINEAR
     * 这个是指定插值的方式，图像缩放之后，肯定像素要进行重新计算的，就靠这个参数来指定重新计算像素的方式，有以下几种：
     * INTER_NEAREST - 最邻近插值
     * INTER_LINEAR - 双线性插值，如果最后一个参数你不指定，默认使用这种方法
     * INTER_AREA - resampling using pixel area relation. It may be a preferred method for image decimation,
     * as it gives moire’-free results. But when the image is zoomed, it is similar to the INTER_NEAREST method.
     * INTER_CUBIC - 4x4像素邻域内的双立方插值
     * INTER_LANCZOS4 - 8x8像素邻域内的Lanczos插值
     */
    cv::Mat resize_image;
    cv::resize(image, resize_image, cv::Size(640, 480), 0, 0, CV_INTER_AREA);
    std::cout << "image.type(): " << image.type() << std::endl;
    std::cout << "resize_image.type(): " << resize_image.type() << std::endl;
    cv::imshow("resize_image", resize_image);
    // 暂停程序,等待一个按键输入 如果不暂停画面会一闪而过
    cv::waitKey(0);
    // /home/q/CLionProjects/5-opencv/imageBasics/resize-ubuntu.png
    std::string path=(argv[2]);
    imwrite(path, resize_image);



    // 也可以这样创建一张图片
    cv::Mat image_zero = cv::Mat::zeros(cv::Size(640, 480), CV_8UC3);
    //设置蓝色背景
    image_zero.setTo(cv::Scalar(100, 0, 0));

    cv::imshow("image_zero", image_zero);
    // 暂停程序,等待一个按键输入,如果不暂停画面会一闪而过
    cv::waitKey(0);

    // 创建一张 三通道 彩色图片
    cv::Mat img_red(300, 400, CV_8UC3, cv::Scalar_<uchar>(0,0,255,0));
    cv::Mat img_green(300, 400, CV_8UC3, cv::Scalar_<uchar>(0,255,0,0));
    cv::Mat img_blue(300, 400, CV_8UC3, cv::Scalar_<uchar>(255,0,0,0));


    // 当Alpha值为0时，该像素是完全透明的，而当Alpha值为255时，则该像素是完全不透明。
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

    // 对于图像还有很多基本的操作,如剪切,旋转,缩放等,限于篇幅就不一一介绍了,请参看OpenCV官方文档查询每个函数的调用方法.
    cv::destroyAllWindows();


    return 0;
}
