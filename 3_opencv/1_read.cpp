#include <opencv2/opencv.hpp>

using namespace cv;

int main(){
	cv::Mat image=cv::imread("./1_read.jpg", CV_LOAD_IMAGE_COLOR);
    cv::namedWindow("girl", CV_WINDOW_AUTOSIZE);
    cv::imshow("girl",image);

    
    std::cout << "图片的行数rows:" << image.rows << std::endl;
    std::cout << "图片的列数cols:" << image.cols << std::endl;
    std::cout << "图片的通道数channels:" << image.channels() << std::endl;
    std::cout << "图片矩阵的维数dims:" << image.dims << std::endl;
    std::cout << "type:" << image.type() << std::endl;
    if (image.type() != CV_8UC1 && image.type() != CV_8UC3) {
        std::cout << "请输入一张彩色图或灰度图." << std::endl;
        return 0;
    }
    std::cout << "depth:" << image.depth() << std::endl;
    std::cout << "elemSize:" << image.elemSize() << std::endl;
    std::cout << "elemSize1:" << image.elemSize1() << std::endl;



    Mat B(image);
    imshow("B", B);

    Mat C;
    C = image;
    imshow("C", C);

    Mat D (image, Rect(0, 0, image.cols / 2, image.rows / 2));
    imshow("D", D);

    Mat E = image(Range::all(), Range(0,300));
    imshow("E", E);

    Mat F = image.clone();
    imshow("F" ,F);

    Mat G ;
    image.copyTo(G);
    cv::namedWindow( "G", CV_WINDOW_AUTOSIZE);
    imshow("G", G);

	cv::waitKey(0);

}