#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

int main() {
    std::cout << "小秋SLAM实战教程公众号后台回复“0704”获取可执行代码" << std::endl;
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
    // 叉乘的计算方法:(x1, x2, x3) x (y1, y2, y3) = (y1z2 - y2z1, z1x2-z2x1, x1y2 -x2y1)
    // Eigen:(1, 2, 1), v2(2, 2, 1); -> (0, 1, -2)
    std::cout << "叉乘 cross " << v1.cross(v2).transpose() << std::endl;

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

    return 0;
}
