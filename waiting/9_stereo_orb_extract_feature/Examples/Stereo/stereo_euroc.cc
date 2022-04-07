#include<iostream>
#include<algorithm>
#include<System.h>
#include<opencv2/core/core.hpp>

using namespace std;

void LoadImages(const string &strPathLeft, const string &strPathRight, const string &strPathTimes,
                vector<string> &vstrImageLeft, vector<string> &vstrImageRight, vector<double> &vTimeStamps);

int main(int argc, char **argv){
    // 1. 加载指定路径下面的数据集
    vector<string> vstrImageLeft;
    vector<string> vstrImageRight;
    vector<double> vTimeStamp;
    LoadImages("../MH_01_easy/mav0/cam0/data",
               "../MH_01_easy/mav0/cam1/data",
               "./MH01.txt",
               vstrImageLeft, vstrImageRight, vTimeStamp);
    const int nImages = vstrImageLeft.size();
    cout << "Images in the sequence: " << nImages << endl;

    // 2. 加载相机参数文件对图像进行去畸变
    // Read rectification parameters
    // https://chunqiushenye.blog.csdn.net/article/details/108703089
    cv::FileStorage fsSettings("./EuRoC.yaml", cv::FileStorage::READ);

    cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
    fsSettings["LEFT.K"] >> K_l;
    fsSettings["RIGHT.K"] >> K_r;

    fsSettings["LEFT.P"] >> P_l;
    fsSettings["RIGHT.P"] >> P_r;

    fsSettings["LEFT.R"] >> R_l;
    fsSettings["RIGHT.R"] >> R_r;

    fsSettings["LEFT.D"] >> D_l;
    fsSettings["RIGHT.D"] >> D_r;

    int rows_l = fsSettings["LEFT.height"];
    int cols_l = fsSettings["LEFT.width"];
    int rows_r = fsSettings["RIGHT.height"];
    int cols_r = fsSettings["RIGHT.width"];

    cout << "Camera Parameters: " << endl;
    cout << "K_l\n" << K_l << endl;
    cout << "K_r\n" << K_r << endl;
    cout << "P_l\n" << P_l << endl;
    cout << "P_r\n" << P_r << endl;
    cout << "R_l\n" << R_l << endl;
    cout << "R_r\n" << R_r << endl;
    cout << "D_l\n" << D_l << endl;
    cout << "D_r\n" << D_r << endl;

    // 图片畸变矫正的映射矩阵mapx、mapy
    //第一个参数cameraMatrix为相机内参矩阵
    //第二个参数distCoeffs为相机畸变矩阵
    //第三个参数R是第一和第二相机坐标之间的旋转矩阵
    //第四个参数newCameraMatrix为校正后的 新的相机内参矩阵
    //第五个参数size为图像尺寸
    //第六个参数定义map1的数据类型，可以是CV_32FC1或者CV_16SC2
    //第七个参数map1和第八个参数map2，map1存储去畸变像素的横坐标 map2存储去畸变像素的纵坐标
    // https://blog.csdn.net/qq_21950671/article/details/114333314
    cv::Mat M1l,M2l,M1r,M2r;
    cv::initUndistortRectifyMap(K_l, D_l, R_l, P_l.rowRange(0,3).colRange(0,3), cv::Size(cols_l,rows_l), CV_32F, M1l, M2l);
    cv::initUndistortRectifyMap(K_r, D_r, R_r, P_r.rowRange(0,3).colRange(0,3), cv::Size(cols_r,rows_r), CV_32F, M1r, M2r);

    // 3. 创建 ORB_SLAM2::System 对象
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM("./EuRoC.yaml");

    // Main loop
    cv::Mat imLeft, imRight, imLeftRect, imRightRect;
    for(int ni=0; ni<nImages; ni++){
        // Read left and right images from file
        imLeft = cv::imread(vstrImageLeft[ni],CV_LOAD_IMAGE_UNCHANGED);
        imRight = cv::imread(vstrImageRight[ni],CV_LOAD_IMAGE_UNCHANGED);

        // 重映射原理：通过移动像素修改图像的外观，这个过程不会修改像素值,而是把每个像素的位置重新映射到新的位置。
        // 第一个参数src畸变图像
        // 第二个参数dst矫正后的图像
        // 第三个参数map1和第四个参数map2，X坐标和Y坐标的映射
        // 第五个参数interpolation图像的插值方式
        // 第六个参数borderMode边界填充方式
        cv::remap(imLeft, imLeftRect, M1l, M2l, cv::INTER_LINEAR);
        cv::remap(imRight, imRightRect, M1r, M2r, cv::INTER_LINEAR);

        // 6. 执行 ORB_SLAM2::System 函数 TrackStereo
        SLAM.TrackStereo(imLeftRect, imRightRect);
    }
    return 0;
}

// C++函数的参数加了const和&有什么作用？ https://blog.csdn.net/qq_21950671/article/details/103528012#t42
void LoadImages(const string &strPathLeft,
                const string &strPathRight,
                const string &strPathTimes,
                vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight,
                vector<double> &vTimeStamps)
{
    ifstream fTimes;
    fTimes.open(strPathTimes.c_str());
    vTimeStamps.reserve(5000);
    vstrImageLeft.reserve(5000);
    vstrImageRight.reserve(5000);
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            // https://chunqiushenye.blog.csdn.net/article/details/98885199
            stringstream ss;
            ss << s;
            vstrImageLeft.push_back(strPathLeft + "/" + ss.str() + ".png");
            vstrImageRight.push_back(strPathRight + "/" + ss.str() + ".png");
            double t;
            ss >> t;
            vTimeStamps.push_back(t/1e9);

        }
    }
}