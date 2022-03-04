//
// Created by q on 2019/11/18.
//
#include <opencv2/opencv.hpp>
#include <vector>

using namespace cv;
using namespace std;

void creatAlphaMat(Mat &mat)
{
    for(int i = 0; i < mat.rows;  i ++)
    {
        for (int j = 0; j < mat.cols; j ++)
        {
            Vec4b&rgba = mat.at<Vec4b>(i, j);
            rgba[0] = UCHAR_MAX;
            rgba[1] = saturate_cast<uchar>((float (mat.cols -j)) / ((float)mat.cols) * UCHAR_MAX);
            rgba[2] = saturate_cast<uchar>((float (mat.rows - i)) / ((float)mat.cols) * UCHAR_MAX);
            rgba[3] = saturate_cast<uchar>(0.5 * (rgba[1] + rgba[2]));
        }
    }
}


int main ()
{
    Mat mat(400, 500, CV_8UC4);
    int step3 = (int)mat.step1();
    cout << "(int)image.step1()= " << step3 << endl;
    int step4 = (int)mat.step;
    cout << "(int)image.step()= " << step4 << endl;
    creatAlphaMat(mat);
    vector<int> compression_params;
    compression_params.push_back(IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(9);
    try{
        imwrite("透视Alpha值图.png", mat, compression_params);
        imshow("生成的PNG图", mat);
        fprintf(stdout, "PNG图片文件alpha数据保存完毕\n可以在工程目录查看由imwrite函数生成的图片\n");
        waitKey(0);
    }
    catch (runtime_error& ex){
        fprintf(stderr, "图像装换成PNG格式发生错误：%s\n", ex.what());
        return 1;
    }
    return 0;

}
