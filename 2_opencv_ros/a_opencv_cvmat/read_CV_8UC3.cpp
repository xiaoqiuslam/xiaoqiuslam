
#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>


int main()
{
    cv::Mat mat =  cv::imread("../rgbd_dataset_freiburg1_xyz_rgb_1305031102.175304.png", -1);
    cv::namedWindow( "image", cv::WINDOW_NORMAL);
    imshow( "image", mat);
    cv::waitKey(0);

    std::cout << "rows " << mat.rows << std::endl;
    std::cout << "cols " << mat.cols << std::endl;
    std::cout << "dims " << mat.dims << std::endl;
    std::cout << "channels() " << mat.channels() << std::endl;
    std::cout << "type() " << mat.type() << std::endl;
    std::cout << "depth() " << mat.depth() << std::endl;
    std::cout << "elemSize() " << mat.elemSize() << std::endl;
    std::cout << "elemSize1() " << mat.elemSize1() << std::endl;
    std::cout << "step " << mat.step << std::endl;
    std::cout << "step1() " << mat.step1() << std::endl;

    /**
     * rows 480
     * cols 640
     * dims 2
     * channels() 3
     * type() 16
     * depth() 0
     * elemSize() 3
     * elemSize1() 1 通道下每个元素的字节数
     * step 1920=640*3 一行像素占用多少个字节
     * step1() 1920 step1==step/elemSize1
     */
}


