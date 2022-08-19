#include <iostream>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "sophus/so3.h"
#include "sophus/se3.h"

int main(int argc, char **argv) {
    /******************************************
    * 		原型					  李代数
    * SO3 (3*3)=="R"		      --> so3 (3*1)
    * SE3 (4*4)=="T"  	          --> se3 (6*1)
    ********************************************/
    // 旋转向量: 第一个参数为旋转角度，第二个参数哪个为1就绕哪轴旋转
    Eigen::Matrix3d rotation_matrix = Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d(0, 0, 1)).toRotationMatrix();
    std::cout << "用沿Z轴转90度的Eigen旋转向量构造Eigen的旋转矩阵:\n" << rotation_matrix << std::endl;

    // 旋转矩阵->李群旋转矩阵SO3
    Sophus::SO3 SO3_rotation_matrix(rotation_matrix);
    std::cout << "用Eigen的旋转矩阵构造Sophus李群旋转矩阵SO3:\n" << SO3_rotation_matrix.matrix() << std::endl;

    // 李群旋转矩阵对数映射->李代数so3
    Eigen::Vector3d so3 = SO3_rotation_matrix.log();
    std::cout << "对数映射求李群SO3的李代数so3 = " << so3.transpose() << std::endl;

    // 李代数指数映射->李群旋转矩阵
    Sophus::SO3 SO3_rotation_matrix_2;
    SO3_rotation_matrix_2 = Sophus::SO3::exp(so3);
    std::cout << "指数映射求李代数so3的李群SO3 = \n" << SO3_rotation_matrix_2.matrix() << std::endl;

    // 李代数向量hat->反对称矩阵
    std::cout << Sophus::SO3::hat(so3) << std::endl;

    // 反对称矩阵vee->李代数向量
    std::cout << Sophus::SO3::vee(Sophus::SO3::hat(so3)).transpose() << std::endl;

    //李代数更新量
    Eigen::Vector3d update_so3(0.0001, 0, 0);

    //更新李群旋转矩阵
    Sophus::SO3 SO3_updated = Sophus::SO3::exp(update_so3) * SO3_rotation_matrix;
    std::cout << "更新李群SO3的旋转矩阵 = \n" << SO3_updated.matrix() << std::endl;


    // 平移向量
    Eigen::Vector3d translation_vector (1, 0, 0);

    // 从R,t构造SE(3)李群变换矩阵
    Sophus::SE3 SE3_transformation_matrix (rotation_matrix, translation_vector);
    std::cout << "用旋转矩阵R,平移向量t构造Sophus表示的李群变换矩阵SE3= \n" << SE3_transformation_matrix.matrix() << std::endl;

    // 李代数变换se3是一个六维向量
    Eigen::Matrix<double, 6, 1> se3 = SE3_transformation_matrix.log();
    std::cout << "李代数se3 是一个六维向量 李群SE3的对数映射求李代数se3 \n" << se3.transpose() << std::endl;

    // 指数映射求李代数se3的李群SE3
    Sophus::SE3 SE3_transformation_matrix_2 = Sophus::SE3::exp(se3);
    std::cout << "指数映射求李代数se3的李群SE3 = \n" << SE3_transformation_matrix_2.matrix() << std::endl;

    // 李代数向量hat->反对称矩阵
    std::cout << "se3 hat = " << std::endl << Sophus::SE3::hat(se3) << std::endl;

    // 反对称矩阵vee->李代数向量
    std::cout << "se3 hat vee = " << Sophus::SE3::vee(Sophus::SE3::hat(se3)).transpose() << std::endl;

    // 更新量
    Eigen::Matrix<double, 6, 1> update_se3;
    update_se3.setZero();
    update_se3(0, 0) = 1e-4d;


    // 增量扰动模型的更新
    Sophus::SE3 SE3_updated = Sophus::SE3::exp(update_se3) * SE3_transformation_matrix;
    std::cout << "SE3 updated = " << std::endl << SE3_updated.matrix() << std::endl;

    return 0;
}