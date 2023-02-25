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
     * ͷ�ļ���#include <sys/types.h>   #include <dirent.h>
     * ���庯����DIR * opendir(const char * name);
     * ����˵����opendir()�����򿪲���name ָ����Ŀ¼, ������DIR*��̬��Ŀ¼��, ��open()����, ��������Ŀ¼�Ķ�ȡ��������Ҫʹ�ô˷���ֵ.
     * ����ֵ���ɹ��򷵻�DIR* ��̬��Ŀ¼��, ��ʧ���򷵻�NULL.
     */
    if(!opendir(TarFilePath.c_str())){
        std::cout << "Ŀ��·��������\n" << std::endl;
        string cmd;
        std::cout << "����Ŀ��·��\n" << std::endl;
        cmd = "mkdir " + TarFilePath;
        system(cmd.c_str());
    }
    else {
        std::cout << "Ŀ��·������\n" << std::endl;
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
             * char ��Ϊ�ַ����ͣ�
             * char *variableName = "string";
             * char��*�ǹ̶����ַ�����ʽ��variableNmae Ϊ�������ƣ�"string" ��Ҫ��ֵ���ַ�����
             * ����ַ�ʹ�� %c������ַ���ʹ�� %s��
             * int rename(const char *old_filename, const char *new_filename)
             * old_filename -- ���� C �ַ�����������Ҫ��������/�ƶ����ļ����ơ�
             * new_filename -- ���� C �ַ������������ļ��������ơ�
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
                    cerr << "���·������û��ͼƬ, ����·���Ƿ���ȷ\n" << vstrImageFilenames[i] << endl;
                }
                else {
                    cout << "ͼƬ��ȡ�ɹ�\n" << endl;
                }
                // imshow("image", image);
                //cout << "���ո�������˳�ͼƬԤ��\n" << endl;
                //cv::waitKey(0);
                //cv::destroyAllWindows();

                char newName[10];
                /**
                 * sprintf()�������ڽ���ʽ��������д���ַ�������ԭ��Ϊ��
                 * int sprintf(char *str, char * format [, argument, ...]);
                 * ��������strΪҪд����ַ�����formatΪ��ʽ���ַ�������printf()������ͬ��argumentΪ������
                 * sprintf()�����Ӧ��֮һĪ���ڰ�������ӡ���ַ����У��磺
                 * sprintf(s, "%d", 123);  //������123��ӡ��һ���ַ���������s��
                 * sprintf(s, "%8x", 4567);  //Сд16���ƣ�����ռ8��λ�ã��Ҷ���
                 */
                sprintf(newName, "%06d.png", i);
                imwrite(rgb_outpath + newName, image);
                cv::Mat image_out = cv::imread(rgb_outpath + newName);
                if (image_out.data == NULL) {
                    cerr << "ͼƬд��ʧ��" << endl;
                } else{
                    cout << "ͼƬд��ɹ� \n" << rgb_outpath + newName<< endl;
                }

            }
            else {
                cout << "��ǰ·������û��ͼƬ, ����·��\n" << endl;
            }
        }
    }
}