#include "myslam/g2o_types.h"

namespace myslam {
// 3D-3D,同时优化位姿和空间点
void EdgeProjectXYZRGBD::computeError() {
  const g2o::VertexSBAPointXYZ *point =
      static_cast<const g2o::VertexSBAPointXYZ *>(_vertices[0]);
  const g2o::VertexSE3Expmap *pose =
      static_cast<const g2o::VertexSE3Expmap *>(_vertices[1]);
   // .map = _r*xyz + _t
  _error = _measurement - pose->estimate().map(point->estimate());
}
// 3D-3D,同时优化位姿和空间点
void EdgeProjectXYZRGBD::linearizeOplus() {
  g2o::VertexSE3Expmap *pose =
      static_cast<g2o::VertexSE3Expmap *>(_vertices[1]);
  // SE3Quat
  // 是g2o中老版本相机位姿的表示，内部使用四元数+平移向量存储位姿，同时支持李代数上的运算
  g2o::SE3Quat T(pose->estimate());
  g2o::VertexSBAPointXYZ *point =
      static_cast<g2o::VertexSBAPointXYZ *>(_vertices[0]);
  Eigen::Vector3d xyz = point->estimate();
  Eigen::Vector3d xyz_trans = T.map(xyz);
  double x = xyz_trans[0];
  double y = xyz_trans[1];
  double z = xyz_trans[2];

  // .rotation() -> return _r
  // .toRotationMatrix() -> Convert the quaternion to a 3x3 rotation matrix. The
  // quaternion is required to
  //  * be normalized, otherwise the result is undefined.
  _jacobianOplusXi = -T.rotation().toRotationMatrix();

  _jacobianOplusXj(0, 0) = 0;
  _jacobianOplusXj(0, 1) = -z;
  _jacobianOplusXj(0, 2) = y;
  _jacobianOplusXj(0, 3) = -1;
  _jacobianOplusXj(0, 4) = 0;
  _jacobianOplusXj(0, 5) = 0;

  _jacobianOplusXj(1, 0) = z;
  _jacobianOplusXj(1, 1) = 0;
  _jacobianOplusXj(1, 2) = -x;
  _jacobianOplusXj(1, 3) = 0;
  _jacobianOplusXj(1, 4) = -1;
  _jacobianOplusXj(1, 5) = 0;

  _jacobianOplusXj(2, 0) = -y;
  _jacobianOplusXj(2, 1) = x;
  _jacobianOplusXj(2, 2) = 0;
  _jacobianOplusXj(2, 3) = 0;
  _jacobianOplusXj(2, 4) = 0;
  _jacobianOplusXj(2, 5) = -1;
}

// 3D-3D,只优化位姿,不优化空间点
void EdgeProjectXYZRGBDPoseOnly::computeError() {
  const g2o::VertexSE3Expmap *pose =
      static_cast<const g2o::VertexSE3Expmap *>(_vertices[0]);
  _error = _measurement - pose->estimate().map(point_);
}

// 3D-3D 只优化位姿,不优化空间点
void EdgeProjectXYZRGBDPoseOnly::linearizeOplus() {
  g2o::VertexSE3Expmap *pose =
      static_cast<g2o::VertexSE3Expmap *>(_vertices[0]);
  g2o::SE3Quat T(pose->estimate());
  Vector3d xyz_trans = T.map(point_);
  double x = xyz_trans[0];
  double y = xyz_trans[1];
  double z = xyz_trans[2];

  _jacobianOplusXi(0, 0) = 0;
  _jacobianOplusXi(0, 1) = -z;
  _jacobianOplusXi(0, 2) = y;
  _jacobianOplusXi(0, 3) = -1;
  _jacobianOplusXi(0, 4) = 0;
  _jacobianOplusXi(0, 5) = 0;

  _jacobianOplusXi(1, 0) = z;
  _jacobianOplusXi(1, 1) = 0;
  _jacobianOplusXi(1, 2) = -x;
  _jacobianOplusXi(1, 3) = 0;
  _jacobianOplusXi(1, 4) = -1;
  _jacobianOplusXi(1, 5) = 0;

  _jacobianOplusXi(2, 0) = -y;
  _jacobianOplusXi(2, 1) = x;
  _jacobianOplusXi(2, 2) = 0;
  _jacobianOplusXi(2, 3) = 0;
  _jacobianOplusXi(2, 4) = 0;
  _jacobianOplusXi(2, 5) = -1;
}

// computeError函数中，描述了error的计算过程，即观测的值，减去计算出的值。
// 3D-2D 只优化位姿，不优化空间点
void EdgeProjectXYZ2UVPoseOnly::computeError() {
  // 顶点v2 constVertexSBAPointXYZ* v2 = static_cast<
  // constVertexSBAPointXYZ*>(_vertices[ 0]); 李群相机位姿v1
  // constVertexSE3Expmap* v1= static_cast<const
  // VertexSE3Expmap*>(_vertices[1]); _vertices[0] 对应的是 VertexSBAPointXYZ
  // 类型的顶点，也就是三维点， _vertices[1] 对应的是VertexSE3Expmap
  // 类型的顶点，也就是位姿pose。 因此前面 1 对应的就应该是 pose，0对应的
  // 应该就是三维点。

  // 注意：_vertices[1]和_vertices[0]，
  // 分别表示“ : public  BaseBinaryEdge<2, Vector2D, VertexSBAPointXYZ,
  // VertexSE3Expmap>”中后面两个绑定点的类型。
  // 因此main函数中，这种边如果想绑定到节点上，即edge->setVertex(0,……)这里的“……”写的就是VertexSBAPointXYZ类型；
  // 如果这种边想绑定到VertexSE3Expmap类型的节点上，edge->setVertex(
  // )中第一个参数就是填1.

  const g2o::VertexSE3Expmap *pose = static_cast<const g2o::VertexSE3Expmap *>(_vertices[0]);
  cout << pose->estimate().map(point_) << endl;
  // Vector2d camera2pixel( const Vector3d& p_c );
  _error = _measurement - camera_->camera2pixel(pose->estimate().map(point_));
}

// 当前顶点的值下，该误差对优化变量的偏导数，也就是我们说的Jacobian
// 3D-2D 只优化位姿，不优化空间点
void EdgeProjectXYZ2UVPoseOnly::linearizeOplus() {
  // 0号顶点为 位姿 类型强转
  g2o::VertexSE3Expmap *pose = static_cast<g2o::VertexSE3Expmap *>(_vertices[0]);
  // 变换矩阵
  g2o::SE3Quat T(pose->estimate());
  //　对点进行变换
  Vector3d xyz_trans = T.map(point_);
   //变换后的 x  y  z
  double x = xyz_trans[0];
  double y = xyz_trans[1];
  double z = xyz_trans[2];
  double z_2 = z * z;

  _jacobianOplusXi(0, 0) = x * y / z_2 * camera_->fx_;
  _jacobianOplusXi(0, 1) = -(1 + (x * x / z_2)) * camera_->fx_;
  _jacobianOplusXi(0, 2) = y / z * camera_->fx_;
  _jacobianOplusXi(0, 3) = -1. / z * camera_->fx_;
  _jacobianOplusXi(0, 4) = 0;
  _jacobianOplusXi(0, 5) = x / z_2 * camera_->fx_;

  _jacobianOplusXi(1, 0) = (1 + y * y / z_2) * camera_->fy_;
  _jacobianOplusXi(1, 1) = -x * y / z_2 * camera_->fy_;
  _jacobianOplusXi(1, 2) = -x / z * camera_->fy_;
  _jacobianOplusXi(1, 3) = 0;
  _jacobianOplusXi(1, 4) = -1. / z * camera_->fy_;
  _jacobianOplusXi(1, 5) = y / z_2 * camera_->fy_;
}
}  // namespace myslam
