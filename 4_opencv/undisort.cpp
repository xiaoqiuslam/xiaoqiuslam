#include <opencv2/opencv.hpp>
#include <string>

using namespace std;

/**
 * https://zhuanlan.zhihu.com/p/74133719
 * https://zhuanlan.zhihu.com/p/137053640
 * https://blog.csdn.net/u013341645/article/details/78710740
 */


int main() {
    /**********************
     * 读取相机内参数和畸变系数
     *********************/
    // 读取相机参数
    const string strSettingPath = "./undistort.yaml";
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];
    // 构造内参数矩阵
    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;

    // 另一种内参数矩阵定义的方法
    //  cv::Matx33f intrinsic_matrix = cv::Matx33f::eye();
    //  intrinsic_matrix(0,0) = fx;
    //  intrinsic_matrix(1,1) = fy;
    //  intrinsic_matrix(0,2) = cx;
    //  intrinsic_matrix(1,2) = cy;

    // 畸变系数 k1, k2, p1, p2
    float k1 = fSettings["Camera.k1"];
    float k2  = fSettings["Camera.k2"];
    float p1 = fSettings["Camera.p1"];
    float p2= fSettings["Camera.p2"];
    // 构造畸变系数矩阵
    cv::Mat DistCoef(4,1,CV_32F);
    DistCoef.at<float>(0) = k1;
    DistCoef.at<float>(1) = k2;
    DistCoef.at<float>(2) = p1;
    DistCoef.at<float>(3) = p2;

    // 另一种定义畸变系数矩阵的方法
    //  cv::Vec4f distortion_coeffs;
    //  distortion_coeffs(0) = k1;
    //  distortion_coeffs(1) = k2;
    //  distortion_coeffs(2) = p1;
    //  distortion_coeffs(3) = p2;

    // 读取带畸变的图像
    cv::Mat image_distorted = cv::imread("./distorted.png", CV_LOAD_IMAGE_GRAYSCALE);
    cv::imshow("image_distorted", image_distorted);
    cv::waitKey();


    /****************************************
     * 使用cv::goodFeaturesToTrack提取图像特征点
     ***************************************/
    const int MAX_CNT = 150;
    const int MIN_DIST = 30;
    vector<cv::Point2f> pts;
    // 提取特征点的图像要是灰度图
    cv::goodFeaturesToTrack(image_distorted, pts, MAX_CNT, 0.01, MIN_DIST);
    /**
     * cv::goodFeaturesToTrack 角点检测
     * 第一个参数是输入图像（8位或32位单通道图）
     * 第二个参数是检测到的所有角点,类型为vector或数组由实际给定的参数类型而定
     * 如果是vector那么它应该是一个包含cv::Point2f的vector对象；如果类型是cv::Mat,那么它的每一行对应一个角点,点的x,y位置分别是两列
     * 第三个参数用于限定检测到的点数的最大值
     * 第四个参数表示检测到的角点的质量水平（通常是0.10到0.01之间的数值,不能大于1.0）
     * 第五个参数用于区分相邻两个角点的最小距离（小于这个距离得点将进行合并）
     * 第六个参数是mask如果指定它的维度必须和输入图像一致且在mask值为0处不进行角点检测
     * 第七个参数是blockSize表示在计算角点时参与运算的区域大小常用值为3但是如果图像的分辨率较高则可以考虑使用较大一点的值
     * 第八个参数用于指定角点检测的方法,如果是true则使用Harris角点检测false则使用Shi Tomasi算法
     * 第九个参数是在使用Harris算法时使用,最好使用默认值0.04
     */
    // 在图像中画出检测到的特征点
    cv::Mat image_distorted_pts = image_distorted.clone();
    for(auto& pt:pts){
        circle(image_distorted_pts, pt, 10, cv::Scalar(255, 0, 0), 1);
        /**
         * 在图像上面画出检测的到的特征点
         * cvCircle(CvArr* img, CvPoint center, int radius, CvScalar color, int thickness=1, int lineType=8, int shift=0)
         * img为源图像指针
         * center为画圆的圆心坐标
         * radius为圆的半径
         * color为设定圆的颜色,B蓝G绿R红
         * thickness如果是正数表示组成圆的线条的粗细程度-1表示圆被填充
         * line_type线条的类型默认是8
         * shift圆心坐标点和半径值的小数点位数
         */
    }
    cv::imshow("image_distorted_pts", image_distorted_pts);
    cv::waitKey();

    // 在原图像上画一个矩形
    cv::Mat image_distorted_rectangle = image_distorted.clone();
    cv::Rect Bbox{338, 141, 23, 57};
    cv::rectangle(image_distorted_rectangle, Bbox, cv::Scalar(255, 0, 0), 2, 1);
    cv::imshow("image_distorted_rectangle", image_distorted_rectangle);
    cv::waitKey();

    int image_Width, image_Height;
    image_Width = fSettings["image.Width"];
    cout << image_Width << endl; // 752
    image_Height = fSettings["image.Height"];
    cout << image_Height << endl; // 480
    cv::Mat image_undistort_diy = cv::Mat(image_Height, image_Width, CV_8UC1);
    for (int v = 0; v < image_Height; v++) {// 计算去畸变后图像的内容
        for (int u = 0; u < image_Width; u++) {
            double x = (u - K.at<float>(0,2)) / K.at<float>(0,0);
            // 按照公式计算点(u,v)对应到畸变图像中的坐标(u_image_undistort_diy, v_image_undistort_diy)
            double y = (v - K.at<float>(1,2)) / K.at<float>(1,1);
            double r = sqrt(x * x + y * y);
            double x_distorted = x * (1 + DistCoef.at<float>(0) * r * r + DistCoef.at<float>(1) * r * r * r * r) +
                                 2 * DistCoef.at<float>(2) * x * y + DistCoef.at<float>(3) * (r * r + 2 * x * x);
            double y_distorted = y * (1 + DistCoef.at<float>(0) * r * r + DistCoef.at<float>(1) * r * r * r * r) +
                                 DistCoef.at<float>(2) * (r * r + 2 * y * y) + 2 * DistCoef.at<float>(3) * x * y;
            double u_distorted = K.at<float>(0,0) * x_distorted + K.at<float>(0,2);
            double v_distorted = K.at<float>(1,1) * y_distorted + K.at<float>(1,2);
            image_undistort_diy.at<uchar>(v, u) = image_distorted.at<uchar>((int) v_distorted, (int) u_distorted);
        }
    }
    cv::imshow("image_undistort_diy", image_undistort_diy);
    cv::waitKey();

    // 对特征点去畸变,并显示N为提取的特征点数量,将N个特征点保存在N*2的mat中
    uint N = pts.size();
    cv::Mat mat_pts(N,2,CV_32F);
    for(int i=0; i<N; i++){
        mat_pts.at<float>(i,0)=pts[i].x;
        mat_pts.at<float>(i,1)=pts[i].y;
    }
    // 调整mat的通道为2,矩阵的行列形状不变
    mat_pts=mat_pts.reshape(2);
    // 对每一个检测到的特征点进行去畸变
    cv::undistortPoints(mat_pts, mat_pts, K, DistCoef, cv::Mat(), K);
    /**
     * void undistortPoints( InputArray src, OutputArray dst, InputArray cameraMatrix, InputArray distCoeffs, InputArray R = noArray(), InputArray P = noArray());
     */
    mat_pts=mat_pts.reshape(1);
    // 存储去几遍校正后的特征点
    for(int i=0; i<N; i++){
        cv::Point2f kp = pts[i];
        kp.x=mat_pts.at<float>(i,0);
        kp.y=mat_pts.at<float>(i,1);
        pts[i] = kp;
    }

    cv::Mat image_undistort_diy_pts = image_undistort_diy.clone();
    // 将去畸变的特征点画在去畸变的图像上
    for(auto& pt:pts){
        circle(image_undistort_diy_pts, pt, 2, cv::Scalar(0, 0, 255), 2);
    }
    cv::imshow("image_undistort_diy_pts", image_undistort_diy_pts);
    cv::waitKey();

    cv::Mat mat_rectangle(4, 2,  CV_32F);
    mat_rectangle.at<float>(0, 0) = Bbox.x;
    mat_rectangle.at<float>(0, 1) = Bbox.y;

    mat_rectangle.at<float>(1, 0) = Bbox.x + Bbox.width;
    mat_rectangle.at<float>(1, 1) = Bbox.y;

    mat_rectangle.at<float>(2, 0) = Bbox.x;
    mat_rectangle.at<float>(2, 1) = Bbox.y + Bbox.height;

    mat_rectangle.at<float>(3, 0) = Bbox.x + Bbox.width;
    mat_rectangle.at<float>(3, 1) = Bbox.y + Bbox.height;

    // 2通道,行列不变
    mat_rectangle = mat_rectangle.reshape(2);
    cv::undistortPoints(mat_rectangle, mat_rectangle, K, DistCoef, cv::Mat(), K);
    // 同样也是调用点的去畸变函数,对矩形方框的四个顶点去畸变并显示
    // 单通道,行列不变
    mat_rectangle = mat_rectangle.reshape(1);

    double MaxX, MaxY;
    Bbox.x = min(mat_rectangle.at<float>(0, 0), mat_rectangle.at<float>(2, 0));
    MaxX   = max(mat_rectangle.at<float>(1, 0), mat_rectangle.at<float>(3, 0));
    Bbox.y = min(mat_rectangle.at<float>(0, 1), mat_rectangle.at<float>(1, 1));
    MaxY   = max(mat_rectangle.at<float>(2, 1), mat_rectangle.at<float>(3, 1));
    Bbox.width = MaxX - Bbox.x;
    Bbox.height = MaxY - Bbox.y;
    cv::Mat image_undistort_diy_rectangle = image_undistort_diy.clone();
    cv::rectangle(image_undistort_diy_rectangle, Bbox, cv::Scalar(0, 0, 255), 2, 1);
    cv::imshow("image_undistort_diy_rectangle", image_undistort_diy_rectangle);
    cv::waitKey();


    cv::Mat mapx = cv::Mat(image_distorted.size(), CV_32FC1);
    cv::Mat mapy = cv::Mat(image_distorted.size(), CV_32FC1);
    cv::Size imageSize(image_Width, image_Height);
    //  const double alpha = 1;
    const double alpha = 0;
    cv::Mat NewCameraMatrix = getOptimalNewCameraMatrix(K, DistCoef, imageSize, alpha, imageSize, 0);
    // 调用opencv函数getOptimalNewCameraMatrix initUndistortRectifyMap remap 去畸变　
    /**
     * Mat cv::getOptimalNewCameraMatrix()函数的功能是"Return the new camera matrix based on the free scaling parameter"
     * 参数含义
     * InputArray 	cameraMatrix,                  // 相机内参矩阵
     * InputArray 	distCoeffs,                    // 相机畸变参数
     * Size 	        imageSize,                     // 图像尺寸
     * double 	        alpha,                     // 缩放比例
     * 当alpha=1 原图像中的所有像素能够得到保留，因此这个时候得到的矫正后的图像是带黑框的
     * 当alpha=0 得到的图像是不带黑色边框的，相对于原图像，此时的图像损失了部分像素
     * Size 	        newImgSize = Size(),           // 校正后的图像尺寸
     * Rect * 	        validPixROI = 0,               // 输出感兴趣区域设置
     * bool 	        centerPrincipalPoint = false   // 可选标志
     *
     * 单目相机 newCameraMatrix可以用cv::getOptimalNewCameraMatrix计算,或者直接与cameraMatrix相等
     * 双目相机 newCameraMatrix一般是用cv::stereoRectify计算
     * 也可以和原图内参数相同,这时候图像比alpha=0的时候还是要损失更多像素,看上去图像更不清晰了
     */

    initUndistortRectifyMap(K, DistCoef, cv::Mat(), NewCameraMatrix, imageSize, CV_16SC2, mapx, mapy);
    /**
     * void cv::initUndistortRectifyMap()函数用于计算原始图像和矫正图像之间的转换关系,将结果以映射的形式表达映射关系存储在map1和map2中
     * 为了完成映射过程, 我们需要获得一些插值为非整数像素的坐标,因为源图像与目标图像的像素坐标不是一一对应的。
     * 一般情况下我们通过重映射来表达每个像素的位置 (x,y)，像这样 : g(x,y) = f ( h(x,y) )
     * g() 是目标图像, f() 是源图像, 而h(x,y) 是作用于 (x,y) 的映射方法函数,在OpenCV用函数remap
     * 这里的map1和map2就是上面cv::initUndistortRectifyMap()计算出来的结果。
     * 参数含义
     * InputArray 	cameraMatrix,     // 原相机内参矩阵
     * InputArray 	distCoeffs,       // 原相机畸变参数
     * InputArray 	R,                // 可选的修正变换矩阵
     * InputArray 	newCameraMatrix,  // 新相机内参矩阵
     * Size 	        size,             // 去畸变后图像的尺寸
     * int 	        m1type,           // 第一个输出的映射(map1)的类型，CV_32FC1 or CV_16SC2
     * OutputArray 	map1,             // 第一个输出映射,存储去畸变像素的横坐标
     * OutputArray 	map2              // 第二个输出映射,存储去畸变像素的纵坐标
     * 第三个参数，InputArray类型的map1，它有两种可能的表示对象：
     * 表示点（x，y）的第一个映射。
     * 表示CV_16SC2 , CV_32FC1 或CV_32FC2类型的X值。
     *
     * 第四个参数，InputArray类型的map2，同样，它也有两种可能的表示对象，而且他是根据map1来确定表示那种对象。
     * 若map1表示点（x，y）时。这个参数不代表任何值。
     * 表示CV_16UC1 , CV_32FC1类型的Y值（第二个值）。
     *
     * 第五个参数，int类型的interpolation,插值方式，之前的resize( )函数中有讲到，需要注意，resize( )函数中提到的
     *
     * INTER_NEAREST - 最近邻插值
     * INTER_LINEAR – 双线性插值（默认值）
     * INTER_CUBIC – 双三次样条插值（逾4×4像素邻域内的双三次插值）
     * INTER_LANCZOS4 -Lanczos插值（逾8×8像素邻域的Lanczos插值）
     * 第六个参数，int类型的borderMode，边界模式，有默认值BORDER_CONSTANT，表示目标图像中“离群点（outliers）”
     *
     * 第七个参数，const Scalar&类型的borderValue，当有常数边界时使用的值，其有默认值Scalar( )，即默认值为0。
     */
    cv::Mat image_undistort_NewCameraMatrix_remap;
    remap(image_distorted, image_undistort_NewCameraMatrix_remap, mapx, mapy, cv::INTER_LINEAR);
    /**
     * void cv::remap()函数功能：把原始图像中某位置的像素映射到矫正后的图像指定位置。
     * 这里的map1和map2就是上面cv::initUndistortRectifyMap()计算出来的结果。
     * 参数含义
     * InputArray    src,                        // 原始图像
     * OutputArray   dst,                        // 矫正图像
     * InputArray    map1,                       // 第一个映射
     * InputArray    map2,                       // 第二个映射
     * int           interpolation,              // 插值方式
     * int           borderMode=BORDER_CONSTANT, // 边界模式
     * const Scalar& borderValue=Scalar()        // 边界颜色，默认Scalar()黑色
     */
    cv::imshow("image_undistort_NewCameraMatrix_remap", image_undistort_NewCameraMatrix_remap);
    cv::waitKey();

    // 用原始相机内参数对图像进行去畸变
    initUndistortRectifyMap(K, DistCoef, cv::Mat(), K, imageSize, CV_16SC2, mapx, mapy);
    cv::Mat image_undistort_K_remap;
    remap(image_distorted, image_undistort_K_remap, mapx, mapy, cv::INTER_LINEAR);
    cv::imshow("image_undistort_K_remap", image_undistort_K_remap);
    cv::waitKey();


    cv::Mat image_undistort_undistort;
    cv::undistort(image_distorted, image_undistort_undistort, K, DistCoef, NewCameraMatrix);// 调用opencv函数 undistort 去畸变
    /**
     * void cv::undistort()函数功能对图像进行畸变矫正,上面是去畸变分步骤的方法,下面是一步到位
     * 如果undistort函数的最后一个参数使用原相机内参，那么得到的结果相当于alpha=0的情况。
     * 如果undistort函数的最后一个参数使用getOptimalNewCameraMatrix计算出来的新矩阵，那么得到损失像素后的图像，当alpha=1时
     * 有多个图片需要矫正推荐组合的方法,这种方法适合图片少的情况
     * 因为initUndistortRectifyMap函数只需要计算一次就行,不需要每次循环都计算
     * undistort函数内部调用了initUndistortRectifyMap和remap
     * 函数参数
     * InputArray 	src,                        // 原始图像
     * OutputArray 	dst,                        // 矫正图像
     * InputArray 	cameraMatrix,               // 原相机内参矩阵
     * InputArray 	distCoeffs,                 // 相机畸变参数
     * InputArray 	newCameraMatrix = noArray() // 新相机内参矩阵
     */
    cv::imshow("image_undistort_undistort", image_undistort_undistort);
    cv::waitKey();
    return 0;
}


