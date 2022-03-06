#ifndef VISUALODOMETRY_H
#define VISUALODOMETRY_H
#include <opencv2/features2d/features2d.hpp>
#include "myslam/common_include.h"
#include "myslam/map.h"
namespace myslam {
class VisualOdometry {
 public:
  // 定义类的智能指针　简化对象创建流程
  typedef shared_ptr<VisualOdometry> Ptr;
  // 枚举表征VO状态，初始化、正常、丢失，分别对应三个不同的代码执行分支
  // 这个控制VO的三种状态   第一帧             顺利跟踪  丢失
  //                     初始化             正常     丢失
  //                     INITIALIZING=-1   OK=0     LOST
  enum VOState { INITIALIZING = -1, OK = 0, LOST };
  // VO状态、地图（关键帧和特征点）、参考帧、当前帧

  // 成员变量声明
  // current VO status
  VOState state_;
  // map with all frames and map points
  Map::Ptr map_;
  // reference key-frame
  Frame::Ptr ref_;
  // current frame
  Frame::Ptr curr_;
  // orb、参考帧3D坐标Point3f、当前帧特征点KeyPoint、当前帧描述子、参考帧描述子、匹配关系
  // orb detector and computer  
  cv::Ptr<cv::ORB> orb_;
  //int num_inliers_;  // number of inlier features in icp

  // parameters
  int num_of_features_;  // number of features
  double scale_factor_;  // scale in image pyramid
  int level_pyramid_;    // number of pyramid levels
  float match_ratio_;    // ratio for selecting  good matches
  int max_num_lost_;     // max number of continuous lost times
  // 最小内点数
  // number of inlier features in icp
  int min_inliers_;  

  // 关键帧筛选标准
  double key_frame_min_rot;    // minimal rotation of two key-frames
  double key_frame_min_trans;  // minimal translation of two key-frames
  double map_point_erase_ratio_;  // remove map point ratio
  

  vector<cv::Point3f> pts_3d_ref_;       // 3d points in reference frame
  // keypoints in current frame
  vector<cv::KeyPoint> keypoints_curr_;
  // descriptor in current frame
  Mat descriptors_curr_;
  Mat descriptors_ref_;                  // descriptor in reference frame
  // 当前帧和参考帧匹配结果存放位置
  // feature matches
  vector<cv::DMatch> feature_matches_;
  // flann matcher
  cv::FlannBasedMatcher matcher_flann_;
  // matched 3d points
  vector<MapPoint::Ptr> match_3dpts_;
  // matched 2d pixels (index of kp_curr)
  vector<int> match_2dkp_index_;
  // the estimated pose of current frame
  SE3 T_c_w_estimated_;
  // number of inlier features in icp
  int num_inliers_;
  // number of lost times
  int num_lost_;
  // 当前帧相对于参考帧的变换矩阵T
  // the estimated pose of current frame
  SE3 T_c_r_estimated_;  
  //内点数(即为匹配出的关键点数量)

  int num_lost_;  // number of lost times

 public:  // functions
  VisualOdometry();
  ~VisualOdometry();

  //  add a new frame 核心，添加新关键帧 
  bool addFrame(Frame::Ptr frame);

 protected:
  // inner operation
  // 提取特征点
  void extractKeyPoints();

  // 计算描述子
  void computeDescriptors();

  // 特征点匹配
  void featureMatching();

  // 位姿估计
  void poseEstimationPnP();
  void optimizeMap();

  // 设置参考帧的 特征点的3D点
  void setRef3DPoints();

  // 添加关键帧
  void addKeyFrame();
  void addMapPoints();

  // 位姿检验模块，匹配点不能太少，运动不能太大
  bool checkEstimatedPose();
  // 检测关键帧
  bool checkKeyFrame();
  double getViewAngle(Frame::Ptr frame, MapPoint::Ptr point);
};
}  // namespace myslam

#endif