#ifndef MYSLAM_G2O_TYPES_H
#define MYSLAM_G2O_TYPES_H

#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

#include "camera.h"
#include "myslam/common_include.h"

namespace myslam {
// optimize the pose and point 同时优化位姿和空间点
class EdgeProjectXYZRGBD
    : public g2o::BaseBinaryEdge<3, Eigen::Vector3d, g2o::VertexSBAPointXYZ,
                                 g2o::VertexSE3Expmap> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  virtual void computeError();
  virtual void linearizeOplus();
  virtual bool read(std::istream& in) {}
  virtual bool write(std::ostream& out) const {}
};

// only to optimize the pose, no point 只优化位姿，不优化空间点
class EdgeProjectXYZRGBDPoseOnly
    : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, g2o::VertexSE3Expmap> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // Error: measure = R*point+t
  virtual void computeError();
  virtual void linearizeOplus();

  virtual bool read(std::istream& in) {}
  virtual bool write(std::ostream& out) const {}

  Vector3d point_;
};

/**
 *  3D-2D 只优化位姿，不优化空间点
 * "2" 是 int 型，表示测量值的维度 （dimension）
 * "Eigen::Vector2d" 表示测量值的数据类型 图像像素坐标x,y的值
 * 该边被绑定在下面类型的顶点上 李群位姿 g2o::VertexSE3Expmap 表示顶点的类型
 */
class EdgeProjectXYZ2UVPoseOnly
    : public g2o::BaseUnaryEdge<2, Eigen::Vector2d, g2o::VertexSE3Expmap> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // 使用当前顶点的值计算的测量值与真实的测量值之间的误差
  virtual void computeError();
  // 当前顶点的值下，该误差对优化变量的偏导数，也就是我们说的Jacobian
  virtual void linearizeOplus();
  // 读盘、存盘函数，一般情况下不需要进行读写操作，仅仅声明一下
  virtual bool read(std::istream& in) {}
  virtual bool write(std::ostream& os) const {};
  Vector3d point_;
  Camera* camera_;
};
}  // namespace myslam
#endif
