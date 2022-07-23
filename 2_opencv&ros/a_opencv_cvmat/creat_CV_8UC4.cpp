
#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>


int main()
{
    cv::Mat mat(4, 6, CV_8UC4);
    CV_Assert(mat.channels() == 4);
    for (int i = 0; i < mat.rows; ++i)
    {
        for (int j = 0; j < mat.cols; ++j)
        {
            cv::Vec4b& bgra = mat.at<cv::Vec4b>(i, j);
            bgra[0] = UCHAR_MAX; // Blue
            bgra[1] = cv::saturate_cast<uchar>((float (mat.cols - j)) / ((float)mat.cols) * UCHAR_MAX); // Green
            bgra[2] = cv::saturate_cast<uchar>((float (mat.rows - i)) / ((float)mat.rows) * UCHAR_MAX); // Red
            bgra[3] = cv::saturate_cast<uchar>(0.5 * (bgra[1] + bgra[2])); // Alpha
        }
    }

    cv::namedWindow( "image", cv::WINDOW_AUTOSIZE );
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


