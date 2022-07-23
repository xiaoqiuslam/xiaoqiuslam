
#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>


int main()
{
    cv::Mat mat =  cv::imread("../MH_01_easy_mav0_cam0_data_1403636579913555456.png", -1);
    cv::namedWindow( "read_image", cv::WINDOW_NORMAL);
    imshow( "read_image", mat);
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
     * cols 752
     * dims 2
     * channels() 1
     * type() 0
     * depth() 0
     * elemSize() 1
     * elemSize1() 1
     * step 752
     * step1() 752
     */
}


