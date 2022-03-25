#ifndef MAP_H
#define MAP_H

#include "myslam/common_include.h"
#include "myslam/frame.h"
#include "myslam/mappoint.h"

namespace myslam {
class Map {
 public:
  // 给类型起一个别名
  typedef shared_ptr<Map> Ptr;
  // map 容器中存储的数据是有序的，而 unordered_map 容器中是无序的。
  // unordered_map 容器和 map
  // 容器一样，以键值对（pair类型）的形式存储数据，存储的各个键值对的键互不相同且不允许被修改。
  // std::unordered_map<std::string, std::string> umap{
  //    {"Python教程","http://python/"},
  //    {"Java教程","http:/java/"},
  //    {"Linux教程","http://linux/"} };
  unordered_map<unsigned long, MapPoint::Ptr> map_points_;  // all landmarks
  unordered_map<unsigned long, Frame::Ptr> keyframes_;      // all key-frames

  Map() {}

  void insertKeyFrame(Frame::Ptr frame);
  void insertMapPoint(MapPoint::Ptr map_point);
};
}  // namespace myslam

#endif  // MAP_H
