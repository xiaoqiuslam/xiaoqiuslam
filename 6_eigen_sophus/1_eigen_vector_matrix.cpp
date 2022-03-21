#include <iostream>
#include <Eigen/Core>


int main( int argc, char** argv ){

    Eigen::Vector3d vector3d;
    vector3d << 3, 2, 1;

    Eigen::Matrix3d matrix3d = Eigen::Matrix3d::Zero();
    std::cout << matrix3d << std::endl;

    Eigen::Matrix<float, 2, 3> matrix23;
    matrix23 << 1, 2, 3, 4, 5, 6;
    std::cout << matrix23 << std::endl;

    // 用()访问矩阵中的元素
    for (int i=0; i<2; i++) {
        for (int j=0; j<3; j++)
            std::cout<<matrix23(i,j)<<" ";
        std::cout << std::endl;
    }

    Eigen::MatrixXd matrixxd = Eigen::MatrixXd::Random(7, 7);
    std::cout << "matrixxd \n" << matrixxd << std::endl;
    std::cout << "The fourth row and 7th column element is " << matrixxd(3, 6) << std::endl;

    Eigen::MatrixXd B2 = matrixxd.block(1, 2, 3, 3);
    std::cout << "Take sub-matrix whose upper left corner is A(1, 2)" << std::endl << B2 << std::endl;

    // take the second column of A
    Eigen::VectorXd a2 = matrixxd.col(1);
    std::cout << "a2\n" << a2 << std::endl;
    // take the first row of B2
    Eigen::VectorXd b2 = B2.row(0);
    std::cout << "b2\n" << b2 << std::endl;

    // take the first three elements of a2
    Eigen::VectorXd c2 = a2.head(3);
    std::cout << "c2\n" << c2 << std::endl;
    // take the last two elements of b2
    Eigen::VectorXd d2 = b2.tail(2);
    std::cout << "d2\n" << d2 << std::endl;

    return 0;
}
