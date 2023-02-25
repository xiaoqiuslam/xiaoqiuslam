
#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>


int main()
{
    cv::Mat mat(4, 6, CV_8UC3);
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

    /**
     * 91 2 79 | 179 52 205 | 236 8 181 | 239 26 248 | 207 218 45 | 183 158 101
     * 102 18 118 | 68 210 139 | 198 207 211 | 181 162 197 | 191 196 40 | 7 243 230
     * 45 6 48 | 173 242 125 | 175 90 63 | 90 22 112 | 221 167 224  | 113 208 123
     * 214 35 229 | 6 143 138 | 98 81 118  | 187 167 140 | 218 178 23 | 43 133 154
     */

    
}


