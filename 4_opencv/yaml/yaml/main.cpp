#include <iostream>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>

using namespace std;
using namespace cv;

int main() {
    /*********************************************************
    * std::ifstream  Input file stream  读取文件
    * std::ofstream  Output file stream 写入文件
    * std::setprecision(9) c++浮点数位数
    * std::fixed 一般的方式输出浮点数 不是科学计数法
    *********************************************************/
    std::ifstream std_in_file("../rgb.txt");
    // 会自动创建文件 std_out_file.txt
    std::ofstream std_out_file_txt("../std_out_file.txt");
    std::ofstream std_out_file_yaml("../std_out_file.yaml");
    vector<string> vImageFilenames;
    vector<double> vTimestamps;
    // 前三行是注释，跳过
    string s0;
    getline(std_in_file, s0);
    getline(std_in_file, s0);
    getline(std_in_file, s0);

    while (!std_in_file.eof()) {
        string s;
        getline(std_in_file, s);
        if (!s.empty()) {
            stringstream ss;
            ss << s;
            double t;
            string sRGB;
            ss >> t;
            // 把时间戳写入文件
            std_out_file_txt << std::setprecision(19) << std::fixed;
            std_out_file_txt << t << std::endl;

            // 虽然可以生成.yaml文件但是格式不是yaml的格式因为没有文件头"%YAML:1.0"
            std_out_file_yaml << t << endl;

            vTimestamps.push_back(t);
            ss >> sRGB;
            vImageFilenames.push_back(sRGB);
        }
    }

    /***********************************************************************************
     * cv::FileStorage cv_file_in("cv_file_read.yaml", FileStorage::READ);    读取文件
     * cv::FileStorage cv_file_out("cv_file_out.yaml", FileStorage::WRITE);　 写入文件
     ***********************************************************************************/
    cv::FileStorage fsSettings("../cv_file_read.yaml", cv::FileStorage::READ);
    // 用std::ofstream生成的yaml文件是不能被cv::FileStorage正常读取的.因为没有文件头"%YAML:1.0"
    // cv::FileStorage fsSettings_("../std_out_file.yaml", cv::FileStorage::READ);

    cv::FileStorage cv_file_write("../cv_file_write.yaml", cv::FileStorage::WRITE);
    float fx = fsSettings["Camera.fx"];
    float fy = fsSettings["Camera.fy"];
    float cx = fsSettings["Camera.cx"];
    float cy = fsSettings["Camera.cy"];
    cv::Mat K = cv::Mat::eye(3, 3, CV_32F);
    K.at<float>(0, 0) = fx;
    K.at<float>(1, 1) = fy;
    K.at<float>(0, 2) = cx;
    K.at<float>(1, 2) = cy;
    // 写入的时候一定要注意键的格式不能有英文的"."
    // 写入的时候一定要同时有键和值
    cv_file_write << "Camera_fx" << fx;
    return 0;
}



