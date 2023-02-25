/**********************************************************************
 * 轨迹文件数据格式:timestamp tx ty tz qx qy qz qw
 * 自定义旋转矩阵和平移向量对轨迹进行变换得到一个新的轨迹
 * 使用ICP算法(取平移作为三维空间点)估计两个轨迹之间的位姿然后将该位姿作用在新轨迹上面
 * 验证ICP算法估计的旋转矩阵和平移向量是否准确(两条轨迹是否重合)
**********************************************************************/
#include <iostream>
#include <pangolin/pangolin.h>
// #include <opencv2/core/core.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include "sophus/se3.hpp"


void DrawTrajectory(std::vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>>pose1, std::vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> pose2);

int main(){
    // std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> poses;
    std::vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> pose_groundtruth;
    std::vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> pose_new;
    std::vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> pose_estimate;
    std::vector<cv::Point3f> pts_new, pts_groundtruth;

    // 定义轴角沿Z轴旋转45°
    Eigen::AngleAxisd rotation_vector(M_PI/4, Eigen::Vector3d(0, 0, 1));
    // 旋转矩阵
    Eigen::Matrix3d rotate = rotation_vector.matrix();
    // 平移向量
    Eigen::Vector3d tranlation(3, -1, 2);
    Sophus::SE3 Transform(rotate,tranlation);
    
    std::ifstream trajectory_file;
    trajectory_file.open("../trajectory.txt");
    if (!trajectory_file) {
        std::cout << "cannot find trajectory file" << std::endl;
        return 1;
    }
    while (!trajectory_file.eof()) {
        double times, x, y, z, qx, qy, qz, qw;
        trajectory_file >> times >> x >> y >> z >> qx >> qy >> qz >> qw;
        Eigen::Quaterniond q(qw, qx, qy, qz);
        Eigen::Vector3d t(x, y, z);
        Sophus::SE3 T(q, t);
        // 读取轨迹数据四元数作为位姿
        pose_groundtruth.push_back(T);
        //　读取轨迹数据平移向量作为三维空间点
        pts_groundtruth.push_back(cv::Point3f(x, y, z));

        // 对位姿进行变换
        Sophus::SE3 T_ = Transform.inverse() * T;
        pose_new.push_back(T_);
        Eigen::Vector3d t1(x, y, z);
        // 三维空间点进行变换
        Eigen::Vector3d t2 = Transform.inverse()*t1;
        pts_new.push_back(cv::Point3f(t2[0], t2[1], t2[2]));
    }
    trajectory_file.close();

    // 用ICP算法估计位姿然后将该位姿作用在新的轨迹上验证准确度
    // center of mass
    cv::Point3f p1, p2;
    int N = pts_groundtruth.size();
    for(int i = 0; i < N; i++){
        p1 += pts_groundtruth[i];
        p2 += pts_new[i];
    }
    // 质心坐标
    p1 = cv::Point3f(cv::Vec3f(p1) / N);
    p2 = cv::Point3f(cv::Vec3f(p2) / N);

    std::vector<cv::Point3f> q1(N);
    std::vector<cv::Point3f> q2(N);
    for(int i = 0; i < N; i++){
        // 去质心坐标
        q1[i] = pts_groundtruth[i] - p1;
        q2[i] = pts_new[i] - p2;
    }
    Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
    for(int i = 0; i < N; i++){
        // 计算矩阵　W  q1 * q2^T
        W += Eigen::Vector3d(q1[i].x, q1[i].y, q1[i].z) * Eigen::Vector3d(q2[i].x, q2[i].y, q2[i].z).transpose();
    }
    std::cout << "W = \n" << W << std::endl;
    // SVD 分解 W 计算R
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();

    if(U.determinant() * V.determinant() < 0){
        for(int x = 0; x < 3; x++)
            U(x, 2) *= -1;
    }
    std::cout << "U=\n"<<U<<std::endl;
    std::cout << "V=\n"<<V<<std::endl;
    Eigen::Matrix3d R_ = U*(V.transpose());
    std::cout << "R extimate = \n" << R_ << std::endl;
    
    // 计算平移 t_ = p - Rp
    Eigen::Vector3d t_ = Eigen::Vector3d(p1.x, p1.y, p1.z) - R_*Eigen::Vector3d(p2.x, p2.y, p2.z);
    std::cout << "t extimate = \n" << t_ << std::endl;

    Sophus::SE3 T_(R_, t_);
    for(int i = 0; i < N; i++){
        // 将新轨迹的位姿态进行变换
        pose_estimate.push_back(T_ * pose_new[i]);
    }
    // 变换前的两个轨迹
    DrawTrajectory(pose_groundtruth, pose_new);
    // ICP 变换后的两个轨迹
    DrawTrajectory(pose_groundtruth, pose_estimate);
    return 0;
}


void DrawTrajectory(std::vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> pose1, std::vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> pose2) {

    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));


    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glLineWidth(2);

        for (size_t i = 0; i < pose1.size() - 1; i++) {
            glColor3f(1 - (float) i / pose1.size(), 0.0f, (float) i / pose1.size());
            glBegin(GL_LINES);
            auto p1 = pose1[i], p2 = pose1[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }

        for (size_t i = 0; i < pose2.size() - 1; i++) {
            glColor3f(1 - (float) i / pose2.size(), 0.0f, (float) i / pose2.size());
            glBegin(GL_LINES);
            auto p1 = pose2[i], p2 = pose2[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }

        pangolin::FinishFrame();
        usleep(5000);
    }
}
