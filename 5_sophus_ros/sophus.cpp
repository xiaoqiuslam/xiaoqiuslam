#include <iostream>
#include <eigen3/Eigen/Core>
#include "sophus/se3.hpp"

int main(int argc, char **argv){

    // 旋转矩阵
    Eigen::Matrix3d eigen_rotation = Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d(0, 0, 1)).toRotationMatrix();
    std::cout << "eigen_rotation \n" << eigen_rotation << std::endl; 

    Sophus::SO3d sophus_rotation(eigen_rotation);
    std::cout << "sophus_rotation \n" << sophus_rotation.matrix() << std::endl;

    Eigen::Vector3d eigen_so3 = sophus_rotation.log();
    std::cout << "eigen_so3 " << eigen_so3.transpose() << std::endl;
    std::cout << "hat eigen_so3 = \n" << Sophus::SO3d::hat(eigen_so3) << std::endl; 
    std::cout << "vee eigen_so3 = \n" << Sophus::SO3d::vee(Sophus::SO3d::hat(eigen_so3)).transpose() << std::endl; 
    
    Sophus::SO3d sophus_rotation_2;
    sophus_rotation_2 = Sophus::SO3d::exp(eigen_so3);
    std::cout << "sophus_rotation_2 \n" << sophus_rotation_2.matrix() << std::endl; 

    Eigen::Vector3d eigen_so3_update(0, 0, 0.0001);
    Sophus::SO3d sophus_rotation_update = Sophus::SO3d::exp(eigen_so3_update) * sophus_rotation;
    std::cout << "sophus_rotation_update \n" << sophus_rotation_update.matrix() << std::endl;

    // 变换矩阵
    Eigen::Vector3d eigen_translation(1, 0, 0);
    Sophus::SE3d sophus_transformation(eigen_rotation, eigen_translation);
    std::cout << "sophus_transformation \n" << sophus_transformation.matrix() << std::endl;

    Eigen::Matrix<double, 6, 1> eigen_se3 = sophus_transformation.log();
    std::cout << "eigen_se3 \n" << eigen_se3.transpose() << std::endl;
    std::cout << "hat eigen_se3 \n" << Sophus::SE3d::hat(eigen_se3) << std::endl;
    std::cout << "vee eigen_se3 \n" << Sophus::SE3d::vee(Sophus::SE3d::hat(eigen_se3)).transpose() << std::endl;

    Sophus::SE3d sophus_transformation_2 = Sophus::SE3d::exp(eigen_se3);
    std::cout << "sophus_transformation_2 \n" << sophus_transformation_2.matrix() << std::endl;

    Eigen::Matrix<double, 6, 1> eigen_se3_update;
    eigen_se3_update.setZero();
    eigen_se3_update(0, 0) = 1e-4;
    Sophus::SE3d sophus_SE3_update = Sophus::SE3d::exp(eigen_se3_update) * sophus_transformation;
    std::cout << "sophus_SE3_update \n" << sophus_SE3_update.matrix() << std::endl;

}