#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

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


    // 三维空间向量的夹角
    Eigen::Vector3d v1(1, 2, 1), v2(2, 2, 1);
    // https://blog.csdn.net/dcrmg/article/details/52416832
    // 点乘的几何意义: 点乘的几何意义是可以用来表征或计算两个向量之间的夹角，以及在b向量在a向量方向上的投影
    // 点乘的计算方法: 1*2+2*2+1*1=2+4+1=7
    // a.b = |a|*|b|cos角度 = 7
    // |a| = 根号（1 + 4 + 1）= ？
    // |b| = 根号（4 + 4 + 1）= ？
    // a.b / |a|*|b| = cos角度 = ？
    std::cout << "dot\n" << v1.dot(v2) << std::endl;


//    // 向量的点乘
//    float vectorDot = x1*x2 + y1*y2 + z1*z2;
//    // 向量1的模
//    double vectorMold1 = Math.sqrt(Math.pow(x1, 2) + Math.pow(y1, 2) + Math.pow(z1, 2));
//    // 向量2的模
//    double vectorMold2 = Math.sqrt(Math.pow(x2, 2) + Math.pow(y2, 2) + Math.pow(z2, 2));
//    // 向量的夹角[0, PI]，当夹角为锐角时，cosθ>0；当夹角为钝角时,cosθ<0
//    double cosAngle = vectorDot / (vectorMold1 * vectorMold2);
//    double radian = Math.acos(cosAngle);
//    return (float) (180 / Math.PI * radian);


    // https://blog.csdn.net/m0_56348460/article/details/117386857
    // 向量的二范数: 欧几里得范数 常用计算向量长度 即向量元素绝对值的平方和再开方
    // 开平方(-20*-20 + 31*31 + -12*-12) = 38.7943
    // 0范数，向量中非零元素的个数。(1  32)
    // 1范数，为绝对值之和。
    // 2范数，就是通常意义上的模。
    // 无穷范数，就是取向量的最大值。
    std::cout << "norm: " << v1.cross(v2).norm() << std::endl;
    std::cout << "v1:\n" << v1 << std::endl;
    std::cout << "v2:\n" << v2 << std::endl;
    std::cout << "v1.transpose(): \n" << v1.transpose() << std::endl;
    std::cout << "v1.transpose() * v2: \n" << v1.transpose() * v2 << std::endl;


    // https://blog.csdn.net/dcrmg/article/details/52416832
    // 叉乘的几何意义: 在三维几何中，向量a和向量b的叉乘结果是一个向量，更为熟知的叫法是法向量，该向量垂直于a和b向量构成的平面。
    // 叉乘的计算方法:(x1, x2, x3) x (y1, y2, y3) = (y1z2 - y2z1, x2z1-z2x1, x1y2 -x2y1)
    // Eigen:(1, 2,  1), v2( 2,  2,  1);    0       1    -2

    std::cout << "cross\n" << v1.cross(v2) << std::endl;


    // https://blog.csdn.net/KYJL888/article/details/106121978
    // https://www.cnblogs.com/yabin/p/6442922.html
    // 指向(y, x)的射线在坐标平面上与x轴正方向之间的角的角度度。
    // 三维空间中两个向量会有一条公垂线(向量叉乘可以求得),
    // 以公垂线为轴,将第二个向量旋转一个角度,使其与第一个向量平行.这个角度即为两向量的夹角.
    double angle = atan2(v1.cross(v2).norm(), v1.transpose() * v2);
    std::cout << "angle: " << angle * 180 / M_PI << "\n";


    return 0;
}
