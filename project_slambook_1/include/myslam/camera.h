#ifndef CAMERA_H
#define CAMERA_H

#include "myslam/common_include.h"

namespace myslam {

// Pinhole RGBD camera model
class Camera {
 public:
  // static std::shared_ptr<Config> config_;
  // 前面直接定义了Config类的智能指针对象
  // 这里 typedef 是 让 Ptr 和 std::shared_ptr<Camera> 等价
  typedef std::shared_ptr<Camera> Ptr;
  // 这里定义的是公开的全局变量 使用范围是这个类的内部
  // Camera intrinsics
  float fx_, fy_, cx_, cy_, depth_scale_;  
  // 默认构造函数声明
  Camera();
  // 带初始化参数的构造函数
  // float fx, float fy, float cx, float cy, float depth_scale = 0
  // 都是函数的形参 形式参数是真正在函数中参与函数代码块中运算的变量
  // 实参是调用函数的时候传递进函数的
  Camera(float fx, float fy, float cx, float cy, float depth_scale = 0)
      : fx_(fx), fy_(fy), cx_(cx), cy_(cy), depth_scale_(depth_scale) {}

  // 世界坐标系 相机坐标系 像素坐标系之间的转换
  // coordinate transform: world, camera, pixel
  Vector3d world2camera(const Vector3d& p_w, const SE3& T_c_w);
  Vector3d camera2world(const Vector3d& p_c, const SE3& T_c_w);
  Vector2d camera2pixel(const Vector3d& p_c);
  Vector3d pixel2camera(const Vector2d& p_p, double depth = 1);
  Vector3d pixel2world(const Vector2d& p_p, const SE3& T_c_w, double depth = 1);
  Vector2d world2pixel(const Vector3d& p_w, const SE3& T_c_w);
};

}  // namespace myslam
#endif  // CAMERA_H
