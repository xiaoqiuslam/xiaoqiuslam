#include <iostream>
#include <eigen3/Eigen/Core>


int main( int argc, char** argv ){

    std::cout << "小秋SLAM实战教程" << std::endl;
    Eigen::Vector3d vector3d;
    vector3d << 1, 2, 3;
    std::cout << "vector3d \n" << vector3d << std::endl;
    std::cout << "vector3d \n" << vector3d.head(2) << std::endl;
    std::cout << "vector3d \n" << vector3d.tail(2) << std::endl;
    vector3d.normalize();
    std::cout << "normalize vector3d \n" << vector3d << std::endl;
    std::cout << "transpose vector3d \n" << vector3d.transpose() << std::endl;

    Eigen::Matrix3d matrix3D;
    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 3; j++){
            matrix3D(i, j) = i*j;
        }
    }

    std::cout << "matrix3D \n" << matrix3D << std::endl;
    std::cout << "element is " << matrix3D(2, 2) << std::endl;
    std::cout << "element is \n" << matrix3D.block(1, 1, 2, 2) << std::endl;
    std::cout << "element is \n" << matrix3D.col(1) << std::endl;
    std::cout << "element is \n" << matrix3D.row(1) << std::endl;

    return 0;
}
