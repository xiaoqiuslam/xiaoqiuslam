#include "myslam/map.h"

// Map类存储关键帧和路标点
namespace myslam {

// 插入关键帧的策略是什么呢？
void Map::insertKeyFrame(Frame::Ptr frame) {
  cout << "Key frame size = " << keyframes_.size() << endl;
  // unordered_map not find return .end()
  if (keyframes_.find(frame->id_) == keyframes_.end()) {
    // 这是插入新的关键帧
    keyframes_.insert(make_pair(frame->id_, frame));
  } else {
    // 这里是替换已经有的关键帧 不是很懂 orz
    keyframes_[frame->id_] = frame;
  }
}

void Map::insertMapPoint(MapPoint::Ptr map_point) {
  // unordered_map not find return .end()
  if (map_points_.find(map_point->id_) == map_points_.end()) {
    map_points_.insert(make_pair(map_point->id_, map_point));
  } else {
    map_points_[map_point->id_] = map_point;
  }
}

}  // namespace myslam
