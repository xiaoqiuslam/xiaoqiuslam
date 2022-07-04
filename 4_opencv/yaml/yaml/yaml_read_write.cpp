#include <yaml-cpp/yaml.h>
#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

int main(int argc, char **argv) {
    // step 1. read yaml
    // 读取文件中的 node
    // 如果要用户yaml 读取 就要添加新的依赖感觉后面可以用这个读取 然后转化成 cv::filestorage 这样opencv就可以读取了,因为用fstream读取还要一行行解析,有些麻烦.
    // 这是 fstream 读取文件的方式
    /*
     * std::fstream file_ifstream_read("../file_ifstream_read.txt");
    vector<string> vImageFilenames;
    vector<double> vTimestamps;
    // 前三行是注释，跳过
    string s0;
    getline(file_ifstream_read, s0);
    getline(file_ifstream_read, s0);
    getline(file_ifstream_read, s0);

    while (!file_ifstream_read.eof()) {
        string s;
        getline(file_ifstream_read, s);
        if (!s.empty()) {
            stringstream ss;
            ss << s;
            double t;
            string sRGB;
            ss >> t;
            vTimestamps.push_back(t);
            cout << t << endl;
            ss >> sRGB;
            vImageFilenames.push_back(sRGB);
        }
    }
     */

    YAML::Node config = YAML::LoadFile("../read_no_yaml_head.yaml");
    cout << "image_width: " << config["image_width"].as<string>() << endl;
    cout << "image.height: " << config["image.height"].as<string>() << endl;
    cout << "age: " << config["age"].as<int>() << endl;
    cout << "skills c++: " << config["skills"]["c++"].as<int>() << endl;
    for (YAML::const_iterator it = config["skills"].begin(); it != config["skills"].end(); ++it) {
        cout << it->first.as<string>() << ": " << it->second.as<int>() << endl;
    }

    cout << "camera_matrix: \n" << config["camera_matrix"] << endl;
    cout << "data: \n" << config["camera_matrix"]["data"] << endl;
    cout << "distortion_coefficients: " << config["distortion_coefficients"] << endl;
    cout << "rectification_matrix: " << config["rectification_matrix"] << endl;
    cout << "projection_matrix: " << config["projection_matrix"] << endl;


    // 创建节点准备写入文件
    YAML::Node node;
    node["key"] = "value";// 创建一个字典 value的值是字符串
    node["number"] = 255;// 创建一个字典 value的值是常量
    node["seq"].push_back("first element");// 创建一个字典字典的值是序列
    node["seq"].push_back("second element");
    cout << node << endl;

    // 创建一个序列
    YAML::Node node_1;
    node_1.push_back("first item");
    node_1.push_back("second_item");
    node_1.push_back("third_item");

    // 序列的元素还是序列
    std::vector<int> v = {1, 3, 5, 7, 9};
    node_1.push_back(v);
    cout << node_1 << endl;

    // Sequence使用下标访问
    auto it = node_1.begin();
    for (; it != node_1.end(); it++)
        std::cout << *(it) << std::endl;

    node_1["key"] = "value";
    //将node_2作为node的一个子节点
    node["node_2"] = node_1;
    //cout << node << endl;
    //给已有node设置一个别名
    node["pointer_to_first_element"] = node["seq"][0];
    //通过node值来删除
    node.remove(node["seq"][0]);
    // 通过key来删除
    node.remove("pointer_to_first_element");
    //std::cout << node << endl;


    // std::fstream 写入文件--------------------------------------------------------------------
    std::fstream fstream("../read_no_head_out.yaml");
    //设置配置文件node数据
    fstream << node << std::endl;
    fstream <<  "camera_matrix:" << std::endl;
    fstream << config["camera_matrix"] << endl;
    fstream << node << std::endl;
    cv::Mat K = cv::Mat::eye(3, 3, CV_32F);
    fstream <<  "K:\n" << K << std::endl;
    fstream.close();


    // cv::filestorage 读取数据-----------------------------------------------------------
    std::string  path = "../read_with_yaml_head.yaml";
    //使用 cv::FileStorage 加载的yaml文件第一行必须是 %YAML:1.0
    cv::FileStorage cv_file_read(path.c_str(), cv::FileStorage::READ);

    if(!cv_file_read.isOpened()){
        cerr << "ERROR: Wrong path at : " << path << endl;
        exit(-1);
    }

    // first method: use (type) operator on FileNode.
    float fx = cv_file_read["Camera.fx"];
    float fy = cv_file_read["Camera.fy"];
    float cx = cv_file_read["Camera.cx"];
    float cy = cv_file_read["Camera.cy"];

    cout << fx << fy << cx << cy << endl;

    // second method: use FileNode::operator >>
    cv::Mat DistCoef(5,1,CV_32F);
    cv_file_read["Camera.k1"] >> DistCoef.at<float>(0);
    cv_file_read["Camera.k2"] >> DistCoef.at<float>(1);
    cv_file_read["Camera.p1"] >> DistCoef.at<float>(2);
    cv_file_read["Camera.p2"] >> DistCoef.at<float>(3);
    cv_file_read["Camera.k3"] >> DistCoef.at<float>(4);
    cout << DistCoef << endl;

    cv::Mat cameraMatrix, camera_matrix;
    cv_file_read["cameraMatrix"] >> cameraMatrix;
    cout << "cameraMatrix\n" << cameraMatrix << endl;
    cv_file_read["camera_matrix"] >> camera_matrix;
    std::cout << camera_matrix << std::endl;

    //迭代器方法
    cv::FileNode features = cv_file_read["features"];
    cv::FileNodeIterator it = features.begin(), it_end = features.end();
    int idx = 0;
    std::vector<uchar> lbpval;

    // iterate through a sequence using FileNodeIterator
    for( ; it != it_end; ++it, idx++ )
    {
        std::cout << "feature #" << idx << ": ";
        std::cout << "x=" << (int)(*it)["x"] << ", y=" << (int)(*it)["y"] << ", lbp: (";
        // you can also easily read numerical arrays using FileNode >> std::vector operator.
        (*it)["lbp"] >> lbpval;
        for( int i = 0; i < (int)lbpval.size(); i++ )
            std::cout << " " << (int)lbpval[i];
        std::cout << ")" << std::endl;
    }
    cv_file_read.release();




    // 这是 filestorage 写入文件的方式 ------------------------------------------------------
    cv::FileStorage cv_file_write("../file_storage_write.yaml", cv::FileStorage::WRITE);
    cv::Mat k = cv::Mat::eye(3, 3, CV_32F);
    // 写入的时候一定要注意键的格式不能有英文的"."
    // 写入的时候一定要同时有键和值
    cv_file_write << "K " << k;
    // step 2. 构造矩阵写入
    Mat cameraMatrix = (Mat_<double>(3, 3) << 1000, 0, 320, 0, 1000, 240, 0, 0, 1);
    Mat distCoeffs = (Mat_<double>(5, 1) << 0.1, 0.01, -0.001, 0, 0);
    cv_file_write << "cameraMatrix" << cameraMatrix << "distCoeffs" << distCoeffs;
    //逐行输入
    cv_file_write << "features" << "[";
    for (int i = 0; i < 4; i++) {
        int x = rand() % 640;
        int y = rand() % 480;
        uchar lbp = rand() % 256;

        cv_file_write << "{:" << "x" << x << "y" << y << "lbp" << "[:";
        for (int j = 0; j < 8; j++)
            cv_file_write << ((lbp >> j) & 1);
        cv_file_write << "]" << "}";
    }
    cv_file_write << "]";
    cv_file_write.release();
    return 0;
}

// enum value { Undefined, Null, Scalar, Sequence, Map };
// 对应未定义、空、标量、序列、字典。
// ofstream         //文件写操作 内存写入存储设备
// ifstream         //文件读操作，存储设备读区到内存中
// fstream          //读写操作，对打开的文件可进行读写操作

