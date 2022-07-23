
#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>


int main()
{
    cv::Mat mat =  cv::imread("../rgbd_dataset_freiburg1_xyz_depth_1305031102.175304.png", -1);
    cv::namedWindow( "depth", cv::WINDOW_NORMAL);
    imshow( "depth", mat);
    cv::waitKey(0);

    std::cout << "rows " << mat.rows << std::endl;
    std::cout << "cols " << mat.cols << std::endl;
    std::cout << "dims " << mat.dims << std::endl;
    std::cout << "channels() " << mat.channels() << std::endl;
    std::cout << "type() " << mat.type() << std::endl;
    std::cout << "depth() " << mat.depth() << std::endl;// CV_16U = 2
    std::cout << "elemSize() " << mat.elemSize() << std::endl;
    std::cout << "elemSize1() " << mat.elemSize1() << std::endl;
    std::cout << "step " << mat.step << std::endl;
    std::cout << "step1() " << mat.step1() << std::endl;

    /**
     * rows 480
     * cols 640
     * dims 2
     * channels() 1
     * type() 2
     * depth() 2
     * elemSize() 2
     * elemSize1() 2
     * step 1280
     * step1() 640
     */
}


