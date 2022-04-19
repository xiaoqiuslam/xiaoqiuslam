// file write
#include <ros/ros.h>
#include "opencv2/opencv.hpp"
#include <time.h>
 
using namespace cv;
using namespace std;
 
int main(int argc, char** argv)
{
    ros::init(argc, argv, "demo_yaml_v2_write");
    ros::NodeHandle n("~");

    string path;

    //运行ros节点后，ros的相对路径是工作包所在工作空间的根目录
    n.param<std::string>("path", path, "./src/ros_demo/demo_yaml/demo_yaml_v2/config/test_write.yaml");

    FileStorage fs(path, FileStorage::WRITE);
 
    fs << "frameCount" << 5;
    time_t rawtime; time(&rawtime);
    fs << "calibrationDate" << asctime(localtime(&rawtime));
    Mat cameraMatrix = (Mat_<double>(3,3) << 1000, 0, 320, 0, 1000, 240, 0, 0, 1);
    Mat distCoeffs = (Mat_<double>(5,1) << 0.1, 0.01, -0.001, 0, 0);
    fs << "cameraMatrix" << cameraMatrix << "distCoeffs" << distCoeffs;
    
    //逐渐输入
    fs << "features" << "[";
    for( int i = 0; i < 3; i++ )
    {
        int x = rand() % 640;
        int y = rand() % 480;
        uchar lbp = rand() % 256;
 
        fs << "{:" << "x" << x << "y" << y << "lbp" << "[:";
        for( int j = 0; j < 8; j++ )
            fs << ((lbp >> j) & 1);
        fs << "]" << "}";
    }
    fs << "]";

    fs.release();
    return 0;
}
