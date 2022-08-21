
#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>


int main()
{
    cv::Mat mat(4, 6, CV_8UC3, cv::Scalar(0, 0, 0));
    randu(mat, cv::Scalar::all(0), cv::Scalar::all(255));
    cv::namedWindow( "image", cv::WINDOW_NORMAL);
    imshow( "image", mat);
    cv::waitKey(0);
    cv::imwrite("../color.png", mat);

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
     * rows 4
     * cols 6
     * dims 2
     * channels() 3
     * type() 16
     * depth() 0
     * elemSize() 3
     * elemSize1() 1
     * step 18
     * step1() 18
     */
}


