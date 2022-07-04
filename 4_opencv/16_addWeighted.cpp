#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace cv;

int main( ){
	Mat image= imread("./dota.jpg");
	namedWindow("dota");
	imshow("dota",image);

    Mat logo= imread("./dota_logo.jpg");
	namedWindow("logo");
	imshow("logo",logo);

	Mat image_roi;
    image_roi= image(Range(376,376+logo.rows),Range(824,824+logo.cols));
    // image_roi= image(Rect(824,376,logo.cols,logo.rows));
    namedWindow("image_roi");
    imshow("image_roi",image_roi);
	addWeighted(image_roi,0.5,logo,0.3,0.,image_roi);
	namedWindow("dota_logo");
	imshow("dota_logo",image);
	waitKey(0);
	return 0;
}
