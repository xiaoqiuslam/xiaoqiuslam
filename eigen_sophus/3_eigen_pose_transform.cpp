#include <iostream>
#include <Eigen/Core>
#include <cmath>
#include <Eigen/Geometry>

int main( int argc, char** argv ){
    // 1. 旋转向量(轴角)
    // 弧度 = 角度*M_PI/180
    // 角度 = 弧度*180/M_PI
    Eigen::AngleAxisd rotation_vector_z(M_PI/2, Eigen::Vector3d ( 0,0,1 ) );
    std::cout << "rotation_vector z" << "angle is: " << rotation_vector_z.angle() * 180 / M_PI << " axis is: " << rotation_vector_z.axis().transpose() << std::endl;

    Eigen::AngleAxisd rotation_vector_y(M_PI/2, Eigen::Vector3d ( 0,1,0 ) );
    std::cout << "rotation_vector y" << "angle is: " << rotation_vector_y.angle() * 180 / M_PI << " axis is: " << rotation_vector_y.axis().transpose() << std::endl;

    Eigen::AngleAxisd rotation_vector(M_PI/2, Eigen::Vector3d ( 1,0,1 ) );
    std::cout << "rotation_vector x " << "angle is: " << rotation_vector.angle() * 180 / M_PI << " axis is: " << rotation_vector.axis().transpose() << std::endl;

    //旋转向量转化为旋转矩阵
    Eigen::Matrix3d rotation_matrix3d;
    rotation_matrix3d = rotation_vector.matrix();
    std::cout<<"rotation_matrix3d \n"<< rotation_matrix3d << std::endl;

    // 2. 旋转矩阵转换为欧拉角,"2" represents the z axis , "0" x axis, "1" y axis
    Eigen::Vector3d euler_angle = rotation_matrix3d.eulerAngles(0, 1, 2);
    // 默认向量都是列向量，所以为了书写和显示方便，转置成行向量
    std::cout << "绕Z轴旋转的角度是 " << euler_angle.z() * 180 / M_PI << std::endl;

    // 3. 欧拉角
    Eigen::Vector3d euler_angle_1 = Eigen::Vector3d(0, 0, M_PI/2);
    std::cout<<"欧拉角: "<< euler_angle_1.transpose() << std::endl;

    // 4. 四元数
    Eigen::Quaterniond quaterniond;
    quaterniond = Eigen::Quaterniond(rotation_vector);
    // Eigen四元数初始化赋值顺序是[w,x,y,z]
    // Eigen四元数内部存储时顺序是[x y z w]
    std::cout << "quaterniond2\n" << quaterniond.coeffs().transpose() << std::endl;

    Eigen::Quaterniond quaterniond_2 = Eigen::Quaterniond(rotation_matrix3d);
    std::cout << "quaterniond_3\n" << quaterniond_2.coeffs().transpose() << std::endl;

    // 5. 变换矩阵 Eigen::Isometry3d或者Eigen::Matrix4d
    Eigen::Isometry3d transform_matrix = Eigen::Isometry3d::Identity();
    transform_matrix.rotate(rotation_matrix3d);
    transform_matrix.pretranslate(Eigen::Vector3d(1, 1, 1));
    std::cout << "Transform matrix = \n" << transform_matrix.matrix() << std::endl;

    Eigen::Quaterniond quaterniond_5 = Eigen::Quaterniond(rotation_vector);
    Eigen::Isometry3d transform_matrix_3 = Eigen::Isometry3d::Identity();
    transform_matrix_3.rotate(quaterniond_5.normalized().toRotationMatrix());
    std::cout << "transform_matrix_3\n"<<transform_matrix_3.matrix() << std::endl;

    // Eigen::Matrix4d构造变换矩阵,对每一个元素赋值的方法是可行的的,我这里采用的是按矩阵块赋值
    Eigen::Matrix4d T2;
    T2.setIdentity();
    T2.block<3,3>(0,0) = rotation_matrix3d;
    T2.topRightCorner(3, 1) = Eigen::Vector3d(1, 1, 1);

    // 6. 坐标变换
    Eigen::Vector3d vector3d_1(1, 0, 0);
    Eigen::Vector3d vector3d_2;
    vector3d_2 = transform_matrix * vector3d_1;
    std::cout << "vector3d_2: " << vector3d_2.transpose() << std::endl;

    return 0;
}