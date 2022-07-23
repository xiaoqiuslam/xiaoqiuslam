#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>

int main( int argc, char** argv ){

    Eigen::VectorXd vectorxd = Eigen::MatrixXd::Random(5,1);
    vectorxd.normalize();
    std::cout << "vectorxd\n" << vectorxd << std::endl;
    Eigen::MatrixXd matrixxd = Eigen::MatrixXd::Random(5,5);
    std::cout << "matrixxd\n" << matrixxd << std::endl;
    std::cout << "转置\n" << matrixxd.transpose() << std::endl;
    std::cout << "各元素和\n" << matrixxd.sum() << std::endl;
    std::cout << "行列式\n" << matrixxd.determinant() << std::endl;
    std::cout << "逆\n" << matrixxd.inverse() << std::endl;
    std::cout << "迹\n" << matrixxd.trace() << std::endl;


    // 解方程 matrixxd * x = vectorxd 这个方程,直接求逆自然是最直接的，但是求逆运算量大
    // 直接求逆
    Eigen::Matrix<double,5,1> x = matrixxd.inverse()*vectorxd;
    std::cout << "x\n" << x << std::endl;
	// QR矩阵分解来求
    x = matrixxd.colPivHouseholderQr().solve(vectorxd);
    std::cout << "x\n" << x << std::endl;
    
    return 0;
}
