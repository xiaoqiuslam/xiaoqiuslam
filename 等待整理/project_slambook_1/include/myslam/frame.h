#ifndef FRAME_H
#define FRAME_H

#include "myslam/camera.h"
#include "myslam/common_include.h"

namespace myslam {

// forward declare 因为在当前类用到了其他的类所以要类的前置声明
class MapPoint;
class Frame {
 public:
  // 实参： 是用户和程序或者函数的之间传递参数的接口
  // 形参： 是类和函数之间参数传递的接口
  // 类的参数： 是类和类之间参数传递的接口
  // 命名空间的参数： 命名空间之间的参数传递
  // 单个文件之间的参数传递： 头文件
  // 多个文件之间参数的传递： 头文件
  // 总结一下就是逐层加上变量的作用域 函数名 类名 命名空间名字 static 借助C语言中文网理解

  // 普通局部变量存储于进程 [栈空间]，使用完毕会立即释放。
  // 静态局部变量使用static修饰符定义，即使在声明时未赋初值，编译器也会把它初始化为0。
  // 且静态局部变量存储于进程的[全局数据区]，即使函数返回，它的值也会保持不变。
  // 变量在全局数据区分配内存空间；编译器自动对其初始化
  // 其作用域为局部作用域，当定义它的函数结束时，其作用域随之结束
  typedef std::shared_ptr<Frame> Ptr;
  unsigned long id_;    // id of this frame
  double time_stamp_;   // when it is recorded
  SE3 T_c_w_;           // transform from world to camera
  Camera::Ptr camera_;  // Pinhole RGBD Camera model
  Mat color_, depth_;   // color and depth image
  // std::vector<cv::KeyPoint>      keypoints_;  // key points in image
  // std::vector<MapPoint*>         map_points_; // associated map points
  bool is_key_frame_;  // whether a key-frame
 public:  // data members
  Frame();
  Frame(long id, double time_stamp = 0, SE3 T_c_w = SE3(),
        Camera::Ptr camera = nullptr, Mat color = Mat(), Mat depth = Mat());
  ~Frame();

  // factory function
  static Frame::Ptr createFrame();

  // find the depth in depth map
  double findDepth(const cv::KeyPoint& kp);

  // Get Camera Center
  Vector3d getCamCenter() const;

  void setPose(const SE3& T_c_w);
  // check if a point is in this frame
  bool isInFrame(const Vector3d& pt_world);
};

}  // namespace myslam

#endif
