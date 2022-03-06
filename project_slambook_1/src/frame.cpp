#include "myslam/frame.h"
/**
 *  typedef std::shared_ptr<Frame> Ptr;
    unsigned long                  id_;            // id of this frame
    double                         time_stamp_;    // when it is recorded
    SE3                            T_c_w_;         // transform from world to
 camera Camera::Ptr                    camera_;        // Pinhole RGBD Camera
 model Mat                            color_, depth_; // color and depth image
    // std::vector<cv::KeyPoint>   keypoints_;     // key points in image
    // std::vector<MapPoint*>      map_points_;    // associated map points
    bool                           is_key_frame_;  // whether a key-frame
 */
namespace myslam {
// 默认构造函数
Frame::Frame() : id_(-1), time_stamp_(-1), camera_(nullptr), is_key_frame_(false){}
// 带参数的构造函数
Frame::Frame(long id, double time_stamp, SE3 T_c_w, Camera::Ptr camera,
             Mat color, Mat depth)
    : id_(id),
      time_stamp_(time_stamp),
      T_c_w_(T_c_w),
      camera_(camera),
      color_(color),
      depth_(depth),
      is_key_frame_(false){}

Frame::~Frame() {}

Frame::Ptr Frame::createFrame() {
  // 这理相当于定义了一个类的全局变量
  static long factory_id = 0;
  // 这里名字挺吓人,无非就是根据类创建了一个有ID的对象,通过变量factory_id使得ID++

  // static std::shared_ptr<Config> config_;
  // config_ = shared_ptr<Config>(new Config)
  // typedef std::shared_ptr<Frame> Ptr;
  // new 和 delete 运算符提供了一种比自动变量和静态变量更灵活的方法,
  // 他们管理了一个内存池,被称为自由存储空间,(堆) 这样做使得通过new生成的变量,
  // 可以在一个函数重分配空间, 另一个函数中delete释放空间
  // 数据的生命周期不完全受程序或函数的生存时间控制
  // 这种做法使的对内存有更大的控制权, 但是内存管理也更加复杂
  return Frame::Ptr(new Frame(factory_id++));
}
// find the depth in depth map
double Frame::findDepth(const cv::KeyPoint& kp) {
  int x = cvRound(kp.pt.x);
  int y = cvRound(kp.pt.y);
  // depth_(depth) 根据彩色图的像素位置寻找深度图中的深度值
  ushort d = depth_.ptr<ushort>(y)[x];
  if (d != 0) {
    return double(d) / camera_->depth_scale_;
  }
  // rgbd相机极有可能某个点没采集到深度值为0的情况
  // check the nearby points
  // 此像素对应的深度值为零的情况下访问下上下左右的像素，然后返回深度值
  // 有点类似 图像的上采样和下采样 就是这个点 没有值
  // 那就用周围的代替，周围也没有那就没有了
  else{
    // check the nearby points
    int dx[4] = {-1, 0, 1, 0};
    int dy[4] = {0, -1, 0, 1};
    for (int i = 0; i < 4; i++) {
      d = depth_.ptr<ushort>(y + dy[i])[x + dx[i]];
      if (d != 0) {
        return double(d) / camera_->depth_scale_;
      }
    }
  }
  //如果还没有，就返回-1.0，表示访问失败
  return -1.0;
}

void Frame::setPose(const SE3& T_c_w) { T_c_w_ = T_c_w; }

/**
 * @brief Get Camera Center　取相机光心
 * T_c_w_ 是在相机坐标系下看世界坐标系上的坐标点,
 * 通过T_c_w_*p_w=p_c可以把世界坐标系上面的点转换(搬)到相机坐标系上面
 *
 * T_c_w_.inverse() 是在世界坐标系下看相机坐标系上的坐标点,
 * 通过T_c_w_.inverse()*p_c=p_w可以把相机坐标系上面的点转换(搬)到世界坐标系上面
 * T_c_w_.inverse()=T_w_c=R^(-1)(-t)
 *
 * 相机坐标系下面的坐标点的坐标(0,0,0)在世界坐标系下的坐标，就是相机的光心在世界坐标系下面的坐标
 * T_c_w_.inverse().translation()　相机坐标系下面的相机光心坐标点在世界坐标系下面的坐标是多少
 * 因为是光心所以只考虑　平移　.translation()
 * @return
 */
Vector3d Frame::getCamCenter() const { return T_c_w_.inverse().translation(); }
/**
 * @brief check if a point is in this frame
 * 这里的原理是世界坐标系下面的点转化到相机坐标系下面最后转换到像素，最后判断像素点是否超出图像的行列也就是边界
 * @param pt_world
 * @return
 */
bool Frame::isInFrame(const Vector3d& pt_world) {
  Vector3d p_cam = camera_->world2camera(pt_world, T_c_w_);
  // cout<<"P_cam = "<<p_cam.transpose()<<endl;
  //这步是取得Z值，小于0直接return false
  if (p_cam(2, 0) < 0) return false;
  Vector2d pixel = camera_->world2pixel(pt_world, T_c_w_);
  // xy值都大于0并且小于color图的行列
  // cout<<"P_pixel = "<<pixel.transpose()<<endl<<endl;
  return pixel(0, 0) > 0 && pixel(1, 0) > 0 && pixel(0, 0) < color_.cols &&
         pixel(1, 0) < color_.rows;
}

}  // namespace myslam
