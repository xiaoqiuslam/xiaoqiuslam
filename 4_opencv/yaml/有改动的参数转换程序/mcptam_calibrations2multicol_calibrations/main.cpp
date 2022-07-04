#include <dirent.h>
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <string>
#include <iomanip>
#include "boost/filesystem.hpp"
#include <dirent.h>
#include <stdio.h>
#include <string.h>

using namespace std;
using namespace cv;


/**
	* for Split string
    *
	* @param s	to splite string
    *
    * @param v	splite result
    *
    * @param c	splite with character
	*/
void split_string(const string &s, vector<string> &v, const string &c) {
    string::size_type a_pos1, a_pos2;
    a_pos1 = 0;
    a_pos2 = s.find(c);
    //string:npos是个特殊值，说明查找没有匹配
    while (string::npos != a_pos2) {
        v.push_back(s.substr(a_pos1, a_pos2 - a_pos1));

        a_pos1 = a_pos2 + c.size();
        a_pos2 = s.find(c, a_pos1);
    }
    if (a_pos1 != s.length())
        v.push_back(s.substr(a_pos1));
}


/**
	* get files vector from dir
	* @param path	dir path
    * @param files	files vector
	*/
void get_files(const string &path, vector<string> &files) {
//    boost::filesystem::path file_path(path);
//    boost::filesystem::recursive_directory_iterator beg_iter(file_path);
//    boost::filesystem::recursive_directory_iterator end_iter;
//    for (; beg_iter != end_iter; ++beg_iter) {
//        if (boost::filesystem::is_directory(*beg_iter)) {
//            continue;
//        } else {
//            std::string strPath = beg_iter->path().string();
//            //cout << "strPath" << strPath << endl;
//            files.push_back(strPath);
//        }
//    }
    DIR *dir;
    struct dirent *dirent_subfile;
    int n = 0;
    if ((dir = opendir(path.c_str())) == NULL)
        printf("can't open %s", path.c_str());
    while (((dirent_subfile = readdir(dir)) != NULL)){
        if (strcmp(dirent_subfile->d_name, ".") == 0 ||strcmp(dirent_subfile->d_name, "..") == 0 )
            continue;
        printf("%s\n", dirent_subfile->d_name);
        files.emplace_back(path + dirent_subfile->d_name);
    }
    closedir(dir);
}

/**
	* 3x3 rotation matrix to Cayley representation
    *
	* @param R	rotation matrix
    *
	* @return   Cayley parameters
	*/

template<typename T>
cv::Matx<T, 3, 1> rot2cayley(const cv::Matx<T, 3, 3> &R) {
    cv::Matx<T, 3, 3> eyeM = cv::Matx<T, 3, 3>::eye();

    cv::Matx<T, 3, 3> C1 = R - eyeM;
    cv::Matx<T, 3, 3> C2 = R + eyeM;
    cv::Matx<T, 3, 3> C = C1 * C2.inv();

    cv::Matx<T, 3, 1> cayley(-C(1, 2), C(0, 2), -C(0, 1));

    return cayley;
}

/**
	* 4x4 homogeneous transformation matrix to Cayley + translation representation
	*
	* @param T	4x4 homogeneous transformation matrix
	*
	* @return c  6x1 Cayley parameters and translation
	*/
template<typename T>
cv::Matx<T, 6, 1> hom2cayley(const cv::Matx<T, 4, 4> &M) {
    cv::Matx<T, 3, 3> R(M(0, 0), M(0, 1), M(0, 2),
                        M(1, 0), M(1, 1), M(1, 2),
                        M(2, 0), M(2, 1), M(2, 2));
    cv::Matx<T, 3, 1> C = rot2cayley(R);

    return cv::Matx<T, 6, 1>(C(0, 0), C(1, 0), C(2, 0),
                             M(0, 3), M(1, 3), M(2, 3));
}


int main(int argc, char **argv) {
    /**
     * /home/q/projects/CLionProjects/mcptam_calibrations2multicol_calibrations/mcptam_calibrations/
     * /home/q/projects/CLionProjects/mcptam_calibrations2multicol_calibrations/pol.txt
     * /home/q/projects/CLionProjects/mcptam_calibrations2multicol_calibrations/poses.dat
     * /home/q/projects/CLionProjects/mcptam_calibrations2multicol_calibrations/MultiCol_calibrations/
     */
    if (argc != 5) {
        cerr << endl << "Mcptam_calibrations_path Mcptam_pol_path Mcptam_pose_path Mcptam2MultiCol_file_path " << endl;
        return 1;
    }
    string Mcptam_calibrations_path = string(argv[1]);
    string Mcptam_pol_path = string(argv[2]);
    string Mcptam_pose_path = string(argv[3]);
    string MultiCol_file = string(argv[4]);

    // 1. creat MultiCamSys_Calibration.yaml
    string p4(MultiCol_file + "MultiCamSys_Calibration.yaml");
    cv::FileStorage cv_file_write(p4, cv::FileStorage::WRITE);

    // 2. read pol.txt and data format transformation
    std::ifstream std_in_file(Mcptam_pol_path);
    assert(std_in_file.is_open());   //若失败,则输出错误消息,并终止程序运行
    vector<string> pols_line;
    string s1, buff;
    while (getline(std_in_file, buff)) {
        if(!buff.empty()) {
            pols_line.push_back(buff);
        }
    }
    vector<vector<string>> pols_nums;
    vector<int> nrinvpols;
    for (int i = 0; i < pols_line.size(); i++) {
        vector<string> v1, v11;
        split_string(pols_line[i], v1, "coefficients: "); //可按多个字符来分隔;
        split_string(v1[1], v11, " ");
        pols_nums.push_back(v11);
        nrinvpols.push_back(v11.size());
    }

    //3. read mcptam_calibrations and data format transformation
    std::vector<string> file_paths;
    get_files(Mcptam_calibrations_path, file_paths);

    int count = file_paths.size();

    // 创建文件夹 然后循环创建三个yaml文件
    system(("mkdir " + MultiCol_file).c_str());
    for (int i = 0; i < count; i++) {
        cout << file_paths[i] << endl;
        string s = file_paths[i];
        vector<string> v, v2;
        split_string(s, v, "/camera");
        string p3;
        if (v.size() != 0);
        {
            string s1 = v[1];
            split_string(s1, v2, ".");
            p3 = (MultiCol_file + "InteriorOrientationFisheye" + v2[0] + ".yaml");
        }
        cv::FileStorage cv_file_write_multi_col(p3, cv::FileStorage::WRITE);
        std::ifstream std_in_file_mcp_tam(file_paths[i]);
        while (!std_in_file_mcp_tam.eof()) {
            string s;
            getline(std_in_file_mcp_tam, s);
            vector<string> v3, v3_1, v_tmp, v4,v5,v6;
            if (!s.empty()) {
                split_string(s, v3, ":");

                if (strcmp(v3[0].c_str(),"image_width") == 0){
                    split_string(v3[1], v3_1, " ");
                    auto image_width = v3_1[1].c_str();
                    cout << "image_width=:" << image_width << endl;
                    cv_file_write_multi_col << "Camera_Iw" << image_width;
                }

                if (strcmp(v3[0].c_str(),"image_height") == 0){
                    split_string(v3[1], v3_1, " ");
                    auto image_height = v3_1[1].c_str();
                    cout << "image_height=:" << image_height << endl;
                    cv_file_write_multi_col << "Camera_Ih" << image_height;
                    cv_file_write_multi_col << "Camera_nrpol" << 5;
                    //cv_file_write_multi_col << "Camera.nrinvpol: " << nrinvpols[v2[0]-1];

                }
                if (strcmp(v3[0].c_str(),"camera_name") == 0){
                    split_string(v3[1], v3_1, " ");
                    auto camera_name = v3_1[1].c_str();
                    cout << "camera_name=:" << camera_name << endl;
                }

                if (strcmp(v3[0].c_str(), "camera_matrix") == 0){
                    auto camera_matrix = v3[0];
                    cout << "camera_matrix=:" << camera_matrix << endl;
                    string s_tmp;
                    getline(std_in_file_mcp_tam, s_tmp);
                    getline(std_in_file_mcp_tam, s_tmp);
                    getline(std_in_file_mcp_tam, s_tmp);
                    split_string(s_tmp, v_tmp, ":");

                    if (strcmp(v_tmp[0].c_str(),"  data") == 0){
                        split_string(v_tmp[1], v4, "[");
                        split_string(v4[1], v5, "]");
                        split_string(v5[0], v6, ", ");
                        for (int i=0;i<v6.size();i++){
                            auto data = v6[i];
                            cout << "data=:" << data << endl;
                        }
                        cv_file_write_multi_col << "Camera.a0: " << v6[0];
                        cv_file_write_multi_col << "Camera.a1: " << v6[1];
                        cv_file_write_multi_col << "Camera.a2: " << v6[2];
                        cv_file_write_multi_col << "Camera.a3: " << v6[3];
                        cv_file_write_multi_col << "Camera.a4: " << v6[5];
                    }
                }



                if (strcmp(v3[0].c_str(),"distortion_coefficients") == 0){
                    auto distortion_coefficients = v3[0];
                    cout << "distortion_coefficients=:" << distortion_coefficients << endl;
                    string s_tmp;
                    getline(std_in_file_mcp_tam, s_tmp);
                    getline(std_in_file_mcp_tam, s_tmp);
                    getline(std_in_file_mcp_tam, s_tmp);
                    split_string(s_tmp, v_tmp, ":");

                    if (strcmp(v_tmp[0].c_str(),"  data") == 0){
                        split_string(v_tmp[1], v4, "[");
                        split_string(v4[1], v5, "]");
                        split_string(v5[0], v6, ", ");
                        for (int i=0;i<v6.size();i++){
                            auto data = v6[i];
                            cout << "data=:" << data << endl;
//                            double a0 = v6[0];
//                            double a1 = -0.0;
//                            double a2 = v6[2];
//                            double a3 = v6[3];
//                            double a4 = v6[4];
                        }
                    }
                }
                else
                    continue;


            }
        }

    }
    return 0;
}



//        double a0 = cam0_node["data"][0].as<double>();
//        double a1 = -0.0;
//        double a2 = cam0_node["data"][1].as<double>();
//        double a3 = cam0_node["data"][2].as<double>();
//        double a4 = cam0_node["data"][3].as<double>();
//
//        YAML::Node cam0_node_1 = camera_calibration_in["camera_matrix"];
//        double c = cam0_node_1["data"][0].as<double>();
//        double d = cam0_node_1["data"][1].as<double>();
//        double u0 = cam0_node_1["data"][2].as<double>();
//        double e = cam0_node_1["data"][3].as<double>();
//        double v0 = cam0_node_1["data"][5].as<double>();
//        cv_file_write_1 << setprecision(20);
////        cv_file_write_1 << "%YAML:1.0" << endl;
////        cv_file_write_1 << "Camera.Iw: " << image_width << endl;
////        cv_file_write_1 << "Camera.Ih: " << image_height << endl << endl;
////        cv_file_write_1 << "Camera.nrpol: " << 5 << endl;
////        cv_file_write_1 << "Camera.nrinvpol: " << nrinvpols[num - 1] << endl << endl;
//
////        cv_file_write_1 << "Camera.a0: " << -a0 << endl;
////        cv_file_write_1 << "Camera.a1: " << -a1 << endl;
////        cv_file_write_1 << "Camera.a2: " << -a2 << endl;
////        cv_file_write_1 << "Camera.a3: " << -a3 << endl;
////        cv_file_write_1 << "Camera.a4: " << -a4 << endl << endl;
//
//        for (vector<string>::size_type j = 0; j != pols_nums[num - 1].size(); ++j) {
//            if (j % 2 == 0)
//                //cv_file_write_1 << "Camera.pol" + to_string(j) + ": " << pols_nums[num - 1][j] << endl;
////            else {
//////                double num_iter_pol = stod(pols_nums[num-1][j]);
////                float num_iter_pol = stof(pols_nums[num - 1][j]);
////                cout << num_iter_pol << endl;
////                num_iter_pol = -num_iter_pol;
////                cv_file_write_1 << "Camera.pol" + to_string(j) + ": " << to_string(num_iter_pol) << endl;
////            }
//
//
////        }
//        std_in_file.close();
//
//        //cv_file_write_1 << endl;
//
////        cv_file_write_1 << "Camera.c: " << c << endl;
////        cv_file_write_1 << "Camera.d: " << d << endl;
////        cv_file_write_1 << "Camera.e: " << e << endl << endl;
////
////        cv_file_write_1 << "Camera.u0: " << u0 << endl;
////        cv_file_write_1 << "Camera.v0: " << v0 << endl << endl;
////
////        cv_file_write_1 << "Camera.mirrorMask: " << 1 << endl << endl;
//
//        //cv_file_write_1.close();
//    }
//
//    //4. write mcptam_calibrations poses to MultiCol_calibrations
//
//    ifstream infile_poses(Mcptam_pose_path);
//    string temp, buff_pose;
//    vector<string> poses_line;
//    vector<string> camera1, camera2, camera3;
//
//
//    vector<string> camera1_cl, camera2_cl, camera3_cl;
//    if (!infile_poses.is_open()) {
//        cout << "can not open the pose file \n" << endl;
//        return -1;
//    }
//
//    while (getline(infile_poses, buff_pose)) {
//        poses_line.push_back(buff_pose);
//    }
//
//    int j = 0;
//    vector<string> poses_line_1, poses_line_2;
//    split_string(poses_line[j], poses_line_1, " ");
//    if (poses_line_1[0] == "camera1");
//    {
//        camera1.push_back(poses_line[j + 1]);
//        camera1.push_back(poses_line[j + 2]);
//        camera1.push_back(poses_line[j + 3]);
//        camera1.push_back("0, 0, 0, 1");
//
//        camera2.push_back(poses_line[j + 6]);
//        camera2.push_back(poses_line[j + 7]);
//        camera2.push_back(poses_line[j + 8]);
//        camera2.push_back("0, 0, 0, 1");
//
//        camera3.push_back(poses_line[j + 11]);
//        camera3.push_back(poses_line[j + 12]);
//        camera3.push_back(poses_line[j + 13]);
//        camera3.push_back("0, 0, 0, 1");
//    }
//    infile_poses.close();
//
//    int rows_camera1 = 4;
//    int columns_camera1 = 4;
//    double num_iter;
//    vector<vector<double> > vector_camera1(rows_camera1, vector<double>(columns_camera1));
//    for (int i = 0; i < rows_camera1; i++) {//初始化
//        split_string(camera1[i], camera1_cl, " ");
//        for (int j = 0; j < columns_camera1; j++) {
//            num_iter = stod(camera1_cl[j]);
//            vector_camera1[i][j] = num_iter;
//
//        }
//        camera1_cl.clear();
//
//    }
//
//    cout << "mcpatm camera1:" << endl;
//
//    for (int i = 0; i < 4; i++) {
//        for (int j = 0; j < 4; j++) {
//            cout << left << setw(3) << vector_camera1[i][j] << " ";
//        }
//        cout << endl;
//    }
//    cout << endl;
//
//    cv::Matx<double, 4, 4> c_camera1(vector_camera1[0][0], vector_camera1[0][1], vector_camera1[0][2],
//                                     vector_camera1[0][3],
//                                     vector_camera1[1][0], vector_camera1[1][1], vector_camera1[1][2],
//                                     vector_camera1[1][3],
//                                     vector_camera1[2][0], vector_camera1[2][1], vector_camera1[2][2],
//                                     vector_camera1[1][3],
//                                     vector_camera1[3][0], vector_camera1[3][1], vector_camera1[3][2],
//                                     vector_camera1[1][3]);
//    cv::Matx<double, 6, 1> camera1_61 = hom2cayley(c_camera1);
//
//    cout << "camera1_61" << std::endl << camera1_61;
//    // 刚刚
////    cv_file_write << "%YAML:1.0" << endl;
////    cv_file_write << "CameraSystem.nrCams: " << 3 << endl << endl;
////    cv_file_write << "# camera back" << endl;
////    cv_file_write << "CameraSystem.cam1_1: " << camera1_61(0) << endl;
////    cv_file_write << "CameraSystem.cam1_2: " << camera1_61(1) << endl;
////    cv_file_write << "CameraSystem.cam1_3: " << camera1_61(2) << endl;
////    cv_file_write << "CameraSystem.cam1_4: " << camera1_61(3) << endl;
////    cv_file_write << "CameraSystem.cam1_5: " << camera1_61(4) << endl;
////    cv_file_write << "CameraSystem.cam1_6: " << camera1_61(5) << endl;
////    cv_file_write << "# camera right " << endl;
//
//    cout << endl;
//    cout << endl;
//
//
//    int rows_camera2 = 4;
//    int columns_camera2 = 4;
//    vector<vector<double> > vector_camera2(rows_camera2, vector<double>(columns_camera2));
//    for (int i = 0; i < rows_camera2; i++) {//初始化
//        split_string(camera2[i], camera2_cl, " ");
//        for (int j = 0; j < columns_camera2; j++) {
//            num_iter = stod(camera2_cl[j]);
//            vector_camera2[i][j] = num_iter;
//        }
//        camera2_cl.clear();
//    }
//
//    cout << "mcpatm camera2:" << endl;
//    for (int i = 0; i < 4; i++) {
//        for (int j = 0; j < 4; j++) {
//            cout << left << setw(3) << vector_camera2[i][j] << " ";
//        }
//        cout << endl;
//
//    }
//    cout << endl;
//
//    cv::Matx<double, 4, 4> c_camera2(vector_camera2[0][0], vector_camera2[0][1], vector_camera2[0][2],
//                                     vector_camera2[0][3],
//                                     vector_camera2[1][0], vector_camera2[1][1], vector_camera2[1][2],
//                                     vector_camera2[1][3],
//                                     vector_camera2[2][0], vector_camera2[2][1], vector_camera2[2][2],
//                                     vector_camera2[2][3],
//                                     vector_camera2[3][0], vector_camera2[3][1], vector_camera2[3][2],
//                                     vector_camera2[3][3]);
//    cv::Matx<double, 6, 1> camera2_61 = hom2cayley(c_camera2);
//    cout << "camera2_61" << std::endl << camera2_61;
//    // 刚刚　
////    cv_file_write << "CameraSystem.cam2_1: " << camera2_61(0) << endl;
////    cv_file_write << "CameraSystem.cam2_2: " << camera2_61(1) << endl;
////    cv_file_write << "CameraSystem.cam2_3: " << camera2_61(2) << endl;
////    cv_file_write << "CameraSystem.cam2_4: " << camera2_61(3) << endl;
////    cv_file_write << "CameraSystem.cam2_5: " << camera2_61(4) << endl;
////    cv_file_write << "CameraSystem.cam2_6: " << camera2_61(5) << endl;
////    cv_file_write << "# camera left " << endl;
//    cout << endl;
//    cout << endl;
//
//
//    int rows_camera3 = 4;
//    int columns_camera3 = 4;
//    vector<vector<double> > vector_camera3(rows_camera3, vector<double>(columns_camera3));
//    for (int i = 0; i < rows_camera3; i++) {
//        split_string(camera3[i], camera3_cl, " ");
//        for (int j = 0; j < columns_camera3; j++) {
//            num_iter = stod(camera3_cl[j]);
//            vector_camera3[i][j] = num_iter;
//        }
//        camera3_cl.clear();
//    }
//
//    cout << "mcpatm camera3:" << endl;
//    for (int i = 0; i < 4; i++) {
//        for (int j = 0; j < 4; j++) {
//            cout << left << setw(3) << vector_camera3[i][j] << " ";
//        }
//        cout << endl;
//    }
//
//    cv::Matx<double, 4, 4> c_camera3(vector_camera3[0][0], vector_camera3[0][1], vector_camera3[0][2],
//                                     vector_camera3[0][3],
//                                     vector_camera3[1][0], vector_camera3[1][1], vector_camera3[1][2],
//                                     vector_camera3[1][3],
//                                     vector_camera3[2][0], vector_camera3[2][1], vector_camera3[2][2],
//                                     vector_camera3[2][3],
//                                     vector_camera3[3][0], vector_camera3[3][1], vector_camera3[3][2],
//                                     vector_camera3[3][3]);
//
//    //cout << "camera3" << std::endl << c_camera3 << endl;
//
//    cv::Matx<double, 6, 1> camera3_61 = hom2cayley(c_camera3);
//    cout << "camera3_61" << std::endl << camera3_61;
//
//    // 刚刚
////    cv_file_write << "CameraSystem.cam3_1: " << camera3_61(0) << endl;
////    cv_file_write << "CameraSystem.cam3_2: " << camera3_61(1) << endl;
////    cv_file_write << "CameraSystem.cam3_3: " << camera3_61(2) << endl;
////    cv_file_write << "CameraSystem.cam3_4: " << camera3_61(3) << endl;
////    cv_file_write << "CameraSystem.cam3_5: " << camera3_61(4) << endl;
////    cv_file_write << "CameraSystem.cam3_6: " << camera3_61(5) << endl;
//
//
////    cv::Matx<double, 4, 4> C2(-0.464389, 0.013224, -0.885532, -0.15137,
////                               0.0310061, 0.999518, -0.00133391, 0.0272226,
////                               0.885088, -0.0280763, -0.464576, -0.0554681,
////                               0,         0,            0,          1);
////
////    cout << endl;
////    cv::Matx <double, 6, 1> new_C2 = hom2cayley(C2);
////    cout << "new_C2" << std::endl << new_C2;
//
//
////    m_c_yaml << setprecision(20);
//
//
//
//
//
//    //cv_file_write.close();