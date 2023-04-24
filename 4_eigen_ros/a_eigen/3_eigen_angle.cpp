#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

int main( int argc, char** argv ){

    Eigen::Vector3d v1(1, 2, 1);
    Eigen::Vector3d v2(2, 2, 1);
    // 叉乘（外积、向量积）
    // 点乘（内积）的几何意义: 点乘的几何意义是可以用来计算两个向量之间的夹角，以及在b向量在a向量方向上的投影
    // 点乘的计算方法: x1*x2 + y1*y2 + z1*z2 = 1*2+2*2+1*1=2+4+1=7
    // a.b = |a|*|b|cos角度 = 7
    // |a| = 根号（1 + 4 + 1）= ？
    // |b| = 根号（4 + 4 + 1）= ？
    // a.b / |a|*|b| = cos角度 = ？
    std::cout << "点乘 dot " << v1.dot(v2) << std::endl;

    // 叉乘的几何意义: 三维几何中向量a和向量b的叉乘结果是一个向量, 是法向量, 垂直于a和b向量构成的平面。
    // 叉乘的计算方法:(x1, x2, x3) x (y1, y2, y3) = (y1z2 - y2z1, x2z1-z2x1, x1y2 -x2y1)
    // Eigen:(1, 2, 1), v2(2, 2, 1); -> (0, 1, -2)
    std::cout << "叉乘 cross\n" << v1.cross(v2) << std::endl;

    //  Compute and KF velocities mRwg estimation
    //  dirG 速度向量
    //  dirG = dirG/dirG.norm();// norm返回的是向量的二范数, 向量元素绝对值的平方和再开方
    //  Eigen::Vector3f gI(0.0f, 0.0f, -1.0f);
    //  Eigen::Vector3f v = gI.cross(dirG);
    //  const float nv = v.norm();// norm返回的是向量的二范数, 向量元素绝对值的平方和再开方
    //  const float cosg = gI.dot(dirG);
    //  const float ang = acos(cosg);
    //  Eigen::Vector3f vzg = v*ang/nv;
    //  Rwg = Sophus::SO3f::exp(vzg).matrix();// 指数映射旋转向量 -> 旋转矩阵


























    

    // 向量的二范数: 向量长度,
    // 开平方(-20*-20 + 31*31 + -12*-12) = 38.7943
    // 指向(y, x)的射线在坐标平面上与x轴正方向之间的角度。
    // 三维空间中两个向量会有一条公垂线(向量叉乘可以求得),
    // 以公垂线为轴, 将第二个向量旋转一个角度, 使其与第一个向量平行.这个角度即为两向量的夹角.
    // 对于任意不同时等于0的实参数x和y，atan2(y,x)所表达的意思是坐标原点为起点，指向(y,x)的射线在坐标平面上与x轴正方向之间的角的角度度。
    // Math.atan2()接受两个参数x和y
    // angel=Math.atan2(y,x)
    // x 指定点的 x 坐标的数字。
    // y 指定点的 y 坐标的数字。
    double angle = atan2(v1.cross(v2).norm(), v1.transpose() * v2);
    std::cout << "angle: " << angle * 180 / M_PI << "\n";

    return 0;
}