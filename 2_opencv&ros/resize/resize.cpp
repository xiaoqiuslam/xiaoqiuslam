
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