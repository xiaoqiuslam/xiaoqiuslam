// opencv_demo
#include <ros/ros.h>
#include "opencv2/opencv.hpp"
 
using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "demo_yaml_v2_read");
    ros::NodeHandle n("~");

    string path;

    //运行ros节点后，ros的相对路径是工作包所在工作空间的根目录
    n.param<std::string>("path", path, "./src/ros_demo/demo_yaml/demo_yaml_v2/config/test_read.yaml");

    //加载opencv相关的yaml文件第一行必须是 %YAML:1.0
    FileStorage fs2(path.c_str(), FileStorage::READ);

    if ( !fs2.isOpened() )
    {
        ROS_INFO("READ File Failed!");
        return 0;
    }
 
    //直接读取
    // first method: use (type) operator on FileNode.
        int frameCount = (int)fs2["frameCount"];
        
        string date;
        // second method: use FileNode::operator >>
        fs2["calibrationDate"] >> date;
        
        Mat cameraMatrix2, distCoeffs2;
        fs2["cameraMatrix"] >> cameraMatrix2;
        fs2["distCoeffs"] >> distCoeffs2;
        
        cout << "frameCount: " << frameCount << endl
            << "calibration date: " << date << endl
            << "camera matrix: " << cameraMatrix2 << endl
            << "distortion coeffs: " << distCoeffs2 << endl;
    //////
    
    //迭代器方法
        FileNode features = fs2["features"];
        FileNodeIterator it = features.begin(), it_end = features.end();
        int idx = 0;
        std::vector<uchar> lbpval;
        
        // iterate through a sequence using FileNodeIterator
        for( ; it != it_end; ++it, idx++ )
        {
            cout << "feature #" << idx << ": ";
            cout << "x=" << (int)(*it)["x"] << ", y=" << (int)(*it)["y"] << ", lbp: (";
            // you can also easily read numerical arrays using FileNode >> std::vector operator.
            (*it)["lbp"] >> lbpval;
            for( int i = 0; i < (int)lbpval.size(); i++ )
                cout << " " << (int)lbpval[i];
            cout << ")" << endl;
        }
    //////
        fs2.release();
}