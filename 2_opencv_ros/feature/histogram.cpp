#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
using namespace cv;

int main(  )
{
    Mat srcImage;
    srcImage=imread("/home/q/slambook2/ch7/1.png");
    imshow( "素材图", srcImage );

    int bins = 256;
    int hist_size[] = {bins};
    float range[] = { 0, 256 };// 从0开始(含)到256(不含)
    const float* ranges[] = { range};
    MatND redHist,grayHist,blueHist;

    int channels_r[] = {0};
    // 进行直方图的计算（红色分量部分） 用 calcHist 函数计算一维直方图
    calcHist( &srcImage, 1, channels_r, Mat(),redHist, 1, hist_size, ranges,true, false );

    // 进行直方图的计算（绿色分量部分）
    int channels_g[] = {1};
    calcHist( &srcImage, 1, channels_g, Mat(),grayHist, 1, hist_size, ranges,true,false );

    // 进行直方图的计算（蓝色分量部分）
    int channels_b[] = {2};
    calcHist( &srcImage, 1, channels_b, Mat(),blueHist, 1, hist_size, ranges,true,false );

    //绘制出三色直方图 参数准备
    double maxValue_red,maxValue_green,maxValue_blue;
    minMaxLoc(redHist, 0, &maxValue_red, 0, 0);
    minMaxLoc(grayHist, 0, &maxValue_green, 0, 0);
    minMaxLoc(blueHist, 0, &maxValue_blue, 0, 0);
    int scale = 1;
    int histHeight=256;
    Mat histImage = Mat::zeros(histHeight,bins*3, CV_8UC3);

    //正式开始绘制
    for(int i=0;i<bins;i++)
    {
        //参数准备
        float binValue_red = redHist.at<float>(i);
        float binValue_green = grayHist.at<float>(i);
        float binValue_blue = blueHist.at<float>(i);
        int intensity_red = cvRound(binValue_red*histHeight/maxValue_red);  //要绘制的高度
        int intensity_green = cvRound(binValue_green*histHeight/maxValue_green);  //要绘制的高度
        int intensity_blue = cvRound(binValue_blue*histHeight/maxValue_blue);  //要绘制的高度

        //绘制红色分量的直方图
        line(histImage,Point(i,histHeight-1),Point(i, histHeight - intensity_red),CV_RGB(255,0,0));
        //绘制绿色分量的直方图
        line(histImage,Point(i+bins,histHeight-1),Point(i+bins, histHeight - intensity_green),CV_RGB(0,255,0));
        //绘制蓝色分量的直方图
        line(histImage,Point(i+bins*2,histHeight-1),Point(i+bins*2, histHeight - intensity_blue),CV_RGB(0,0,255));
    }
    imshow( "图像的RGB直方图", histImage );
    waitKey(0);
    return 0;
}