#ifndef MAPPOINT_H
#define MAPPOINT_H
#include "myslam/common_include.h"
namespace myslam {

class Frame;
class MapPoint {
 public:
  typedef shared_ptr<MapPoint> Ptr;
  // ID
  unsigned long id_;
  static unsigned long factory_id_;
  // wheter a good point
  bool good_;
  // Position in world
  Vector3d pos_;
  // Normal of viewing direction
  Vector3d norm_;
  // Descriptor for matching
  Mat descriptor_;
  // key-frames that can observe this point
  list<Frame*> observed_frames_;

  // being an inliner in pose estimation
  int matched_times_;
  // being visible in current frame
  int visible_times_;  

  // int observed_times_;  // being observed by feature matching algo.
  // int correct_times_;   // being an inliner in pose estimation

  // 构造函数
  MapPoint();
  // 带参数的构造函数
  // MapPoint(long id, Vector3d position, Vector3d norm);
  MapPoint(unsigned long id, const Vector3d& position, const Vector3d& norm,
           Frame* frame = nullptr, const Mat& descriptor = Mat());


  inline cv::Point3f getPositionCV() const {
    return cv::Point3f(pos_(0, 0), pos_(1, 0), pos_(2, 0));
  }
  // factory function
  static MapPoint::Ptr createMapPoint();

  static MapPoint::Ptr createMapPoint(const Vector3d& pos_world,
                                      const Vector3d& norm_,
                                      const Mat& descriptor, Frame* frame);
};
}  // namespace myslam

#endif
