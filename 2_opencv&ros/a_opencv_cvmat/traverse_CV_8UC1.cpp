#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>


int main()
{
    cv::Mat mat(4, 6, CV_8UC1);
    randu(mat, cv::Scalar::all(0), cv::Scalar::all(255));
    cv::namedWindow( "creat_image", cv::WINDOW_NORMAL);
    imshow( "creat_image", mat);
    cv::waitKey(0);

    for(int y = 0; y < mat.rows; y++){
        uchar* data = mat.ptr<uchar>(y);
        for(int x = 0; x < mat.cols * mat.channels(); x++){
            std::cout << float(data[x]) << " ";
        }
        std::cout << std::endl;
    }

    /*
     * 91 | 2 | 79 | 179 | 52 | 205
     * 236 | 8 | 181 | 239 | 26 | 248
     * 207 | 218 | 45 | 183 | 158 | 101
     * 102 | 18 | 118 | 68 | 210 | 139
     */
}


