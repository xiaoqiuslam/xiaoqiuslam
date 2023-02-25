
#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>


int main()
{
    cv::Mat mat =  cv::imread("../ubuntu.png", -1);
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
     * rows 674
     * cols 1200
     * dims 2
     * channels() 4
     * type() 24
     * depth() 0
     * elemSize() 4
     * elemSize1() 1
     * step 4800
     * step1() 4800
     */
}


