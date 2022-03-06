#include "myslam/mappoint.h"
#include "myslam/common_include.h"
/**
 *  unsigned long        id_;            // ID
    static unsigned long factory_id_;    // factory id
    bool                 good_;          // wheter a good point
    Vector3d             pos_;           // Position in world
    Vector3d             norm_;          // Normal of viewing direction
    Mat                  descriptor_;    // Descriptor for matching
    list<Frame*>    observed_frames_;    // key-frames that can observe this
 point
    int                  matched_times_; // being an inliner in pose estimation
    int                  visible_times_; // being visible in current frame
 */
namespace myslam {
// 无参数的构造函数
MapPoint::MapPoint()
    : id_(-1),
      good_(true),
      pos_(Vector3d(0, 0, 0)),
      norm_(Vector3d(0, 0, 0)),
      visible_times_(0),
      matched_times_(0) {}

MapPoint::MapPoint(long unsigned int id, const Vector3d& position,
                   const Vector3d& norm, Frame* frame, const Mat& descriptor)
    : id_(id),
      good_(true),
      pos_(position),
      norm_(norm),
      visible_times_(1),
      matched_times_(1),
      descriptor_(descriptor) {
  observed_frames_.push_back(frame);
}

// 无参数的createMapPoint()函数
MapPoint::Ptr MapPoint::createMapPoint() {
  return MapPoint::Ptr(
      new MapPoint(factory_id_++, Vector3d(0, 0, 0), Vector3d(0, 0, 0)));
}

// 有参数的createMapPoint()函数
MapPoint::Ptr MapPoint::createMapPoint(const Vector3d& pos_world,
                                       const Vector3d& norm,
                                       const Mat& descriptor, Frame* frame) {
  return MapPoint::Ptr(
      new MapPoint(factory_id_++, pos_world, norm, frame, descriptor));
}

unsigned long MapPoint::factory_id_ = 0;
}  // namespace myslam
