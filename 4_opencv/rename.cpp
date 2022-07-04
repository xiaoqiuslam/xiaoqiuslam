#include <iostream>
#include <fstream>
#include <string>
#include <iomanip>
#include <dirent.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;

void rgb_dataset_fr1_desk_with_rgb_txt(const string &strFile, const string &rgb_path, const string &rgb_outpath);


void rgb_dataset_fr1_desk_without_rgb_txt(const string &rgb_path, const string &rgb_outpath);

void CheckTarFilePath(const string &TarFilePath);

int main (int argc, char** argv) {
    if(argc != 2){
        cerr << endl << "Usage: path_to_sequence" << endl;
        return 1;
    }
    string strFile = string(argv[1])+"/rgb.txt";
    string rgb_outpath_with_rgb_txt = string(argv[1])+"rgb_with/";
    CheckTarFilePath(rgb_outpath_with_rgb_txt);
    string rgb_outpath_without_rgb_txt = string(argv[1])+"rgb_without/";
    CheckTarFilePath(rgb_outpath_without_rgb_txt);
    //rgb_dataset_fr1_desk_with_rgb_txt(strFile, argv[1], rgb_outpath_with_rgb_txt);
    rgb_dataset_fr1_desk_without_rgb_txt(argv[1], rgb_outpath_without_rgb_txt);

}

void CheckTarFilePath(const string &TarFilePath) {
    /**
     * 头文件：#include <sys/types.h>   #include <dirent.h>
     * 定义函数：DIR * opendir(const char * name);
     * 函数说明：opendir()用来打开参数name 指定的目录, 并返回DIR*形态的目录流, 和open()类似, 接下来对目录的读取和搜索都要使用此返回值.
     * 返回值：成功则返回DIR* 型态的目录流, 打开失败则返回NULL.
     */
    if(!opendir(TarFilePath.c_str())){
        std::cout << "目标路径不存在\n" << std::endl;
        string cmd;
        std::cout << "创建目标路径\n" << std::endl;
        cmd = "mkdir " + TarFilePath;
        system(cmd.c_str());
    }
    else {
        std::cout << "目标路径存在\n" << std::endl;
    }
    closedir(opendir(TarFilePath.c_str()));
}

void rgb_dataset_fr1_desk_with_rgb_txt(const string &strFile, const string &rgb_path, const string &rgb_outpath) {
    std::ifstream f;
    f.open(strFile.c_str());
    // skip first three lines
    std::string s0;
    getline(f,s0);
    getline(f,s0);
    getline(f,s0);
    int cnt = 1;
    while(!f.eof()) {
        std::string s;
        getline(f, s);
        if(!s.empty()){
            std::stringstream ss;
            ss << s;
            double t;
            std::string sRGB;
            ss >> t >> sRGB;
            std::string oldname = rgb_path + sRGB;
            int result;
            std::string newname;
            std::stringstream name_stream;
            name_stream << rgb_outpath << std::setw(4) << std::setfill('0') << cnt << ".png";
            name_stream >> newname;
            std::cout << newname << std::endl;
            /**
             * char 称为字符类型，
             * char *variableName = "string";
             * char和*是固定的字符串形式，variableNmae 为变量名称，"string" 是要赋值的字符串。
             * 输出字符使用 %c，输出字符串使用 %s。
             * int rename(const char *old_filename, const char *new_filename)
             * old_filename -- 这是 C 字符串，包含了要被重命名/移动的文件名称。
             * new_filename -- 这是 C 字符串，包含了文件的新名称。
             */
            result = rename(oldname.c_str() , newname.c_str());
            if ( result == 0 )
                puts ( "File successfully renamed" );
            else
                perror( "Error renaming file ");
        }
        cnt++;
    }
}


void rgb_dataset_fr1_desk_without_rgb_txt(const string &rgb_path, const string &rgb_outpath){
    vector<cv::String> vstrImageFilenames;
    cv::glob(rgb_path+"/rgb",vstrImageFilenames,false);
    int size = vstrImageFilenames.size();
    if(size == 0){
        cout<<"empty file."<<endl;
    }
    else {
        for (int i = 0; i < size; i++) {
            if (vstrImageFilenames[i].find("png") != cv::String::npos) {
                cv::Mat image = imread(vstrImageFilenames[i], CV_LOAD_IMAGE_UNCHANGED);
                if (image.data == NULL) {
                    cerr << "这个路径下面没有图片, 请检查路径是否正确\n" << vstrImageFilenames[i] << endl;
                }
                else {
                    cout << "图片读取成功\n" << endl;
                }
                // imshow("image", image);
                //cout << "按空格键可以退出图片预览\n" << endl;
                //cv::waitKey(0);
                //cv::destroyAllWindows();

                char newName[10];
                /**
                 * sprintf()函数用于将格式化的数据写入字符串，其原型为：
                 * int sprintf(char *str, char * format [, argument, ...]);
                 * 【参数】str为要写入的字符串；format为格式化字符串，与printf()函数相同；argument为变量。
                 * sprintf()最常见的应用之一莫过于把整数打印到字符串中，如：
                 * sprintf(s, "%d", 123);  //把整数123打印成一个字符串保存在s中
                 * sprintf(s, "%8x", 4567);  //小写16进制，宽度占8个位置，右对齐
                 */
                sprintf(newName, "%06d.png", i);
                imwrite(rgb_outpath + newName, image);
                cv::Mat image_out = cv::imread(rgb_outpath + newName);
                if (image_out.data == NULL) {
                    cerr << "图片写入失败" << endl;
                } else{
                    cout << "图片写入成功 \n" << rgb_outpath + newName<< endl;
                }

            }
            else {
                cout << "当前路径下面没有图片, 请检查路径\n" << endl;
            }
        }
    }
}