#include "myslam/camera.h"
#include "myslam/config.h"

namespace myslam {

// 默认构造函数定义 注意这里的 {}
Camera::Camera() {
  fx_ = Config::get<float>("camera.fx");
  fy_ = Config::get<float>("camera.fy");
  cx_ = Config::get<float>("camera.cx");
  cy_ = Config::get<float>("camera.cy");
  depth_scale_ = Config::get<float>("camera.depth_scale");
}

// 传引用类型的参数 在内存调用方便 具有更优的效果
Vector3d Camera::world2camera(const Vector3d& p_w, const SE3& T_c_w) {
  return T_c_w * p_w;
}

Vector3d Camera::camera2world(const Vector3d& p_c, const SE3& T_c_w) {
  return T_c_w.inverse() * p_c;
}

Vector2d Camera::camera2pixel(const Vector3d& p_c) {
  // 归一化平面　p_c = [u, v, 1]　
  return Vector2d(fx_ * p_c(0, 0) / p_c(2, 0) + cx_,
                  fy_ * p_c(1, 0) / p_c(2, 0) + cy_);
}

Vector3d Camera::pixel2camera(const Vector2d& p_p, double depth) {
  return Vector3d((p_p(0, 0) - cx_) * depth / fx_,
                  (p_p(1, 0) - cy_) * depth / fy_, depth);
}

Vector2d Camera::world2pixel(const Vector3d& p_w, const SE3& T_c_w) {
  return camera2pixel(world2camera(p_w, T_c_w));
}

Vector3d Camera::pixel2world(const Vector2d& p_p, const SE3& T_c_w,
                             double depth) {
  return camera2world(pixel2camera(p_p, depth), T_c_w);
}
}  // namespace myslam
