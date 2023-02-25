#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>

int main() {
    // 构造位姿变换矩阵:这个变换矩阵可以把世界坐标系转到R1坐标系
    Eigen::Quaterniond q1(0.35, 0.2, 0.3, 0.1); // 四元数表示的旋转矩阵
    std::cout << "q1 = " << std::endl << q1.x() << std::endl << q1.y() << std::endl << q1.z() << std::endl << q1.w() << std::endl;
    /*
    * q1 =
    * 0.2
    * 0.3
    * 0.1
    * 0.35
    */
    q1.normalize(); // 可以看到归一化后数字的精度和大小都会发生变化
    std::cout << "q1 = " << std::endl << q1.x() << std::endl << q1.y() << std::endl << q1.z() << std::endl << q1.w() << std::endl;
    /*
     * q1 =
     * 0.39036
     * 0.58554
     * 0.19518
     * 0.68313
     */
    // 位姿的四元数表示和平移向量转化为欧式变换矩阵
    Eigen::Isometry3d T1w(q1);
    Eigen::Vector3d t1(0.3, 0.1, 0.1); // 平移向量
    T1w.pretranslate(t1);
    // 输出位姿矩阵信息
    std::cout << "T1w: " << std::endl << T1w.matrix() << std::endl;
    /*
     * T1w:
     *  0.238095   0.190476   0.952381        0.3
     *  0.72381    0.619048  -0.304762        0.1
     * -0.647619   0.761905   0.00952381      0.1
     *  0          0          0               1
     */
    // R1坐标系下点p1的坐标
    Eigen::Vector3d p1(0.5, 0, 0.2);


    // 构造位姿变换矩阵:这个变换矩阵可以把世界坐标系转到R2坐标系
    Eigen::Quaterniond q2(-0.5, 0.4, -0.1, 0.2); // 四元数表示位姿 旋转矩阵
    q2.normalize(); // 归一化
    Eigen::Isometry3d T2w(q2);
    Eigen::Vector3d t2(-0.1, 0.5, 0.3); // 平移向量
    T2w.pretranslate(t2);
    std::cout << "T2w: " << std::endl << T2w.matrix() << std::endl;



    // Trw 下标从左往右读是r坐标系变换到w坐标系
    // Trw 下标从右往左读是w坐标系下的坐标变换到r坐标系下的坐标

    // T1w.inverse() * p1 把R1坐标系的坐标p1转化为世界坐标系下的坐标
    // T2w * T1w.inverse() * p1 世界坐标系下面的坐标p1转化为R2坐标系下的坐标p2
    Eigen::Vector3d p2 = T2w * T1w.inverse() * p1;

    std::cout << std::endl << "p1: " << p1.transpose() << std::endl;
    std::cout << std::endl << "p2: " << p2.transpose() << std::endl;
    return 0;
}