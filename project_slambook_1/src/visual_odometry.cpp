#include "myslam/visual_odometry.h"
#include <algorithm>
#include <boost/timer.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "myslam/config.h"
#include "myslam/g2o_types.h"
namespace myslam {
// : 初始化列表，在创建对象的时候调用
VisualOdometry::VisualOdometry()
    : state_(INITIALIZING),
      ref_(nullptr),
      curr_(nullptr),
      map_(new Map),
      num_lost_(0),
      num_inliers_(0),
      matcher_flann_(new cv::flann::LshIndexParams(5, 10, 2)) {
  // 特征点数量
  num_of_features_ = Config::get<int>("number_of_features");
  // 图像金字塔比例尺
  scale_factor_ = Config::get<double>("scale_factor");
  // 金字塔层数
  level_pyramid_ = Config::get<int>("level_pyramid");
   // 占比
  match_ratio_ = Config::get<float>("match_ratio");
  // 最大连续丢失帧数
  max_num_lost_ = Config::get<float>("max_num_lost");
   // 最少的内点数量
  min_inliers_ = Config::get<int>("min_inliers");
  // 关键帧旋转 OZ  
  key_frame_min_rot = Config::get<double>("keyframe_rotation");
  // 关键帧变换 OZ
  key_frame_min_trans = Config::get<double>("keyframe_translation");
  map_point_erase_ratio_ = Config::get<double>("map_point_erase_ratio");
  orb_ = cv::ORB::create(num_of_features_, scale_factor_, level_pyramid_);
}

VisualOdometry::~VisualOdometry() {}

// 这个函数是关键函数， 会频繁调用状态机
// 传入的frame包含了rgb，depth，camera，时间这些参数
bool VisualOdometry::addFrame(Frame::Ptr frame) {
  //根据VO状态执行不同的指令
  switch (state_) {
    // 默认为执行初始化,第一帧
    case INITIALIZING: {
      // 将VO状态更改为OK
      state_ = OK;
      // 将当前帧和参考帧都设置为传入帧
      curr_ = ref_ = frame;
      // 把Frame插入map中 添加关键帧
      // map 包含关键帧 和关键帧上面的特征点
      // map_->insertKeyFrame(frame);
      // extract features from first frame and add them into map
      extractKeyPoints();
      // 计算特征点描述子
      computeDescriptors();
      // compute the 3d position of features in ref frame
      // setRef3DPoints();
      addKeyFrame();  // the first frame is a key-frame
      break;
    }

    // 执行参考帧和当前帧的位姿计算
    case OK: {
      // 将传入帧设置为当前帧
      curr_ = frame;
      curr_->T_c_w_ = ref_->T_c_w_;
      // 提取特征点  
      extractKeyPoints();
       // 计算描述子 
      computeDescriptors();
      // 特征匹配 
      featureMatching();
      // 位姿估算   
      poseEstimationPnP();
      // 位姿检测合格则将当前帧位姿设置为估算值，并添加关键帧（将传入帧中特征点加入map匹配点集合）
      // a good estimation
      if (checkEstimatedPose() == true)  
      {
        // T_c_w = T_c_r*T_r_w
        // curr_->T_c_w_ = T_c_r_estimated_ * ref_->T_c_w_; 
        //当前帧为参考帧
        //ref_ = curr_;
        //参考帧特征点的3D坐标,补全depth数据
        // setRef3DPoints();
        //如果计算的好，将经过BA优化的（当前帧和地图之间的）变换矩阵给curr_->T_c_w_
        curr_->T_c_w_ = T_c_w_estimated_;
        //更新地图，添加和删除地图中的点
        optimizeMap();
        //统计连续的估计运动差的帧的个数，只要出现一帧估计的好，计数器就清零
        // num_lost重置为0
        num_lost_ = 0;
         // is a key-frame
        if (checkKeyFrame() == true) 
        {
          addKeyFrame();
        }
      }
      // 否则num_lost自增，超过最大值则将VO状态设置为LOST，结束程序
      // bad estimation due to various reasons
      else
      {
        //这一帧的运动估计差，则加1,当连续max_num_lost_帧都差的时候，则估计失败
        num_lost_++;
        if (num_lost_ > max_num_lost_) {
          state_ = LOST;
        }
        return false;
      }
      break;
    }
    //匹配失败状态
    case LOST: {
      // num_lost自增，超过最大值则将VO状态设置为LOST，结束程序
      cout << "vo has lost." << endl;
      break;
    }
  }

  return true;
}

// 当前帧提取特征点
void VisualOdometry::extractKeyPoints() {
  boost::timer timer;
   //从curr_->color_中检测出keypoint放入keypoints_curr_
  orb_->detect(curr_->color_, keypoints_curr_);
  cout << "extract keypoints cost time: " << timer.elapsed() << endl;
}

// 当前帧计算描述子
void VisualOdometry::computeDescriptors() {
  boost::timer timer;
  // 计算curr_->color_中检测出的keypoint的descriptor
  // 输出描述子，查看是不是行向量，但是太大了，显示一个矩阵
  orb_->compute(curr_->color_, keypoints_curr_, descriptors_curr_);
  cout << "descriptor computation cost time: " << timer.elapsed() << endl;
}

//特征点匹配
void VisualOdometry::featureMatching() {
  boost::timer timer;
  // match desp_ref and desp_curr, use OpenCV's brute force match
  // matches----->匹配结果存放地点(筛选前))
  vector<cv::DMatch> matches;
  /**
  cv::BFMatcher matcher(cv::NORM_HAMMING);
  // 将当前帧所有特征点对应的描述子，与所有特征点集合中描述子，进行匹配，FLANN是快速最近邻搜索并计算最小近似度
  // 加算前后两组描述子的 复合要求的匹配对
  matcher.match(descriptors_ref_, descriptors_curr_, matches);
  // 关键点筛选：select the best matches
  // 找到matches数组中最小距离，赋值给min_dist std::min_element 用于寻找范围
  // [first, last) 中的最小元素。
  // 前2个参数指定容器的范围，第3个参数是比较函数，为可选参数。
  // 返回值为指向范围 [first, last) 中最小元素的迭代器。
  // 若范围中有多个元素等价于最小元素，则返回指向首个这种元素的迭代器。若范围为空则返回
  // last 。
  */


  // 4
  // select the candidates in map
  //建立一个保存描述子的map矩阵，保存匹配需要地图点的描述子，
  //因为描述子是一个行向量
  Mat desp_map;
  vector<MapPoint::Ptr> candidate;
  for (auto& allpoints : map_->map_points_) {
    MapPoint::Ptr& p = allpoints.second;
    // check if p in curr frame image
    // 检测这个点（世界坐标系中）是否在当前帧的视野之内
    if (curr_->isInFrame(p->pos_)) {
      // add to candidate
      p->visible_times_++;
      candidate.push_back(p);
      desp_map.push_back(p->descriptor_);
    }
  }

  //采用新的匹配方法FlannBasedMatcher(最近邻近似匹配)，而不是暴力匹配BFMatcher；
  //这步匹配中，第一个参数由原来的参考帧，变成了上面定义的desp_map地图，进行匹配。
  //也就是当前帧直接和地图进行匹配
  // 3 matcher_flann_.match(descriptors_ref_, descriptors_curr_, matches);
  matcher_flann_.match(desp_map, descriptors_curr_, matches);
  // select the best matches
  // select the best matches
  //找出容器matches中最小的元素的指针或迭代器
  // distance变量返回两个描述子之间的距离
  // 这里用到了lambda表达式，也就是这里面的第三个参数
  // 可以隐藏返回值的类型，会根据return自动确定，这里又加上了
  float min_dis =
      std::min_element(matches.begin(), matches.end(),
                       [](const cv::DMatch& m1, const cv::DMatch& m2) {
                         return m1.distance < m2.distance;
                       })
          ->distance;


  // 3
  //feature_matches_.clear();
  ////根据距离筛选特征点
  //for (cv::DMatch& m : matches) {
  //  if (m.distance < max<float>(min_dis * match_ratio_, 30.0)) {
  //    //符合条件的匹配放入feature_matches_
  //    feature_matches_.push_back(m);
  //  }
  //}

  //保存已经匹配的MapPoint（MapPoint的指针类型）
  match_3dpts_.clear();
  //保存已经匹配的2d关键点的索引
  match_2dkp_index_.clear();
  for (cv::DMatch& m : matches) {
    //如果描述子之间的距离小于一个值（30和min_dis*match_ratio_中较大的），则表示匹配成功
    if (m.distance < max<float>(min_dis * match_ratio_, 30.0)) {
      //会匹配成功很多，但只有一部分符合条件，将符合条件的这些特征点放入容器
      // queryIdx表示参考帧的匹配成功的索引；trainIdx表示当前帧的匹配成功的索引
      match_3dpts_.push_back(candidate[m.queryIdx]);
      match_2dkp_index_.push_back(m.trainIdx);
    }
  }
  //平均每帧的好的匹配点大概100~200个左右
  cout << "good matches: " << match_3dpts_.size() << endl;
  cout << "match cost time: " << timer.elapsed() << endl;

  //cout << "good matches: " << feature_matches_.size() << endl;
  //cout << "match cost time: " << timer.elapsed() << endl;
}
//这里删除了setRef3DPoints()函数，用不到了，因为我们当前帧不需要和参考帧对比，而是直接和地图对比
//利用和地图匹配得到的信息（match_3dpts_）来进行pnp估计位姿
// 参考帧3D点设置
// pnp需要参考帧3D,当前帧2D，所以当前帧迭代为参考帧时，需要加上depth数据
// 每轮循环结束，都需要把当前帧变成参考帧，这时候就需要把当前帧的坐标的2D形式转化成3D形式
// void VisualOdometry::setRef3DPoints() {
//   // select the features with depth measurements
//   pts_3d_ref_.clear();
//   descriptors_ref_ = Mat();
//   for (size_t i = 0; i < keypoints_curr_.size(); i++) {
//     // 根据特征点的坐标在深度图里面找到深度值
//     double d = ref_->findDepth(keypoints_curr_[i]);
//     if (d > 0) {
//       // 像素坐标到相机坐标系3D坐标
//       Vector3d p_cam = ref_->camera_->pixel2camera(
//           Vector2d(keypoints_curr_[i].pt.x, keypoints_curr_[i].pt.y), d);
//       // 构造Point3f，3D坐标
//       pts_3d_ref_.push_back(cv::Point3f(p_cam(0, 0), p_cam(1, 0), p_cam(2, 0)));
//       // 参考帧描述子，将当前帧描述子按行放进去。
//       descriptors_ref_.push_back(descriptors_curr_.row(i));
//     }
//   }
// }

// 使用pts_3d_ref_ keypoints_curr_、feature_matches_、相机参数　PnP估算相机位姿T
void VisualOdometry::poseEstimationPnP() {
  // construct the 3d 2d observations
  // 参考帧3D坐标和当前帧2D坐标（需要.pt像素坐标）
  vector<cv::Point3f> pts3d;
  vector<cv::Point2f> pts2d;
  // // 遍历每一个匹配的特征点
  // for (cv::DMatch m : feature_matches_) {
  //   // 用到前面的方法 3D-2D 匹配获取位置
  //   // vector<cv::Point3f>     pts_3d_ref_;  3d points in reference
  //   // frame // query=ref
  //   pts3d.push_back(pts_3d_ref_[m.queryIdx]);
  //   // vector<cv::KeyPoint>    keypoints_curr_;     // keypoints in current
  //   // frame // train=curr
  //   pts2d.push_back(keypoints_curr_[m.trainIdx].pt);
  // }

  //取出当前帧匹配成功的2d点
  for (int index : match_2dkp_index_) {
    pts2d.push_back(keypoints_curr_[index].pt);
  }
  //取出参考帧（实际上是map里面）的匹配成功的点
  for (MapPoint::Ptr pt : match_3dpts_) {
    // getPositionCV为inline函数，因为短小且经常被调用
    // getPositionCV函数得到3d点的坐标，因为MapPoint类里面包含了很多信息，世界坐标系的坐标也在其中
    pts3d.push_back(pt->getPositionCV());
  }


  // 构造像机内参矩阵K
  Mat K = (cv::Mat_<double>(3, 3) << ref_->camera_->fx_, 0, ref_->camera_->cx_,
           0, ref_->camera_->fy_, ref_->camera_->cy_, 0, 0, 1);
  //旋转、平移、内点数组
  //得到rvec就是旋转矢量，输出的inlier就是局内点的索引，也就是符合模型的数据的索引，是一个列向量
  //一般的pnp我们用的是最小二乘法，而这里用的是ransac
  /*参数定义
   * objectPoints，要匹配的3d空间点数组
   * objectPoints，要匹配的2d图像点数组
   * cameraMatrix，相机内参矩阵
   * distCoeffs，相机畸变矩阵
   * rvec，旋转向量输出承接矩阵
   * tvec，平移向量输出承接矩阵
   * 后面的参数跟ransac算法有关。倒是都有默认值
   * useExtrinsicGuess，迭代初始值是否定为提供的rvec和tvec值，这里没有提供，所以用false
   * iterationsCount，迭代次数，ransac算法所必须的迭代次数
   * reprojectionError，重投影误差。ransac算法迭代时也必须要规定的误差阀值，来确定是否为内点。
   * confidence，置信度。ransac算法每次用于更新迭代次数的参数。一般固定选为0.995
   * inliers，内点输出承接数组
   */
  Mat r
  Mat rvec, tvec, inliers;
  // 将匹配点及内参K等参数传入cv::solvePnPRansac求解得到外参R t 内点inliers
  cv::solvePnPRansac(pts3d, pts2d, K, Mat(), rvec, tvec, false, 100, 4.0, 0.99,
                     inliers);
  // 内点是匹配出的特征点中真正被位姿估计函数使用的那些点 局内点数量
  num_inliers_ = inliers.rows;
  //输出符合模型的数据的个数
  cout << "pnp inliers: " << num_inliers_ << endl;
  // 参考这个理解:T_c_w_是在相机坐标系下看世界坐标系上的坐标点,通过T_c_w_*p_w=p_c可以把世界坐标系上面的点转换(搬)到相机坐标系上面
  T_c_w_estimated_ = SE3(
      SO3(rvec.at<double>(0, 0), rvec.at<double>(1, 0), rvec.at<double>(2, 0)),
      Vector3d(tvec.at<double>(0, 0), tvec.at<double>(1, 0),
               tvec.at<double>(2, 0)));

  // 上面是没有优化 R|t  T 下面用g2o优化
  // using bundle adjustment to optimize the pose
  //上面ransac会输出符合模型的点的索引，下面对这些符合模型的点进行BA
  // using bundle adjustment to optimize the pose
  typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 2>> Block;
  Block::LinearSolverType* linearSolver =
      new g2o::LinearSolverDense<Block::PoseMatrixType>();
  Block* solver_ptr = new Block(linearSolver);
  g2o::OptimizationAlgorithmLevenberg* solver =
      new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
  g2o::SparseOptimizer optimizer;
  optimizer.setAlgorithm(solver);

  g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap();
  pose->setId(0);
  // pose->setEstimate(g2o::SE3Quat(T_c_r_estimated_.rotation_matrix(),
  //                                T_c_r_estimated_.translation()));
  pose->setEstimate(g2o::SE3Quat(T_c_w_estimated_.rotation_matrix(),
                                 T_c_w_estimated_.translation()));
  optimizer.addVertex(pose);

  // edges
  for (int i = 0; i < inliers.rows; i++) {
    int index = inliers.at<int>(i, 0);
    // 3D -> 2D projection
    EdgeProjectXYZ2UVPoseOnly* edge = new EdgeProjectXYZ2UVPoseOnly();
    edge->setId(i);
    edge->setVertex(0, pose);
    edge->camera_ = curr_->camera_.get();
    // index表示局内点的索引，也就是符合模型的点的索引
    edge->point_ = Vector3d(pts3d[index].x, pts3d[index].y, pts3d[index].z);
    edge->setMeasurement(Vector2d(pts2d[index].x, pts2d[index].y));
    edge->setInformation(Eigen::Matrix2d::Identity());
    optimizer.addEdge(edge);
    // set the inlier map points
    //这个参数用于计算匹配率
    match_3dpts_[index]->matched_times_++;
  }

  optimizer.initializeOptimization();
  optimizer.optimize(10);

  // T_c_r_estimated_ =
  //经过BA优化之后的TCW
  T_c_w_estimated_ =
      SE3(pose->estimate().rotation(), pose->estimate().translation());

  cout << "T_c_w_estimated_: " << endl << T_c_w_estimated_.matrix() << endl;
}

// 位姿检验模块，匹配点不能太少，运动不能太大
bool VisualOdometry::checkEstimatedPose() {
  // check if the estimated pose is good
  // 如果内点数太少则返回false，然后num_lost自增，自增到最大值时结束VO计算
  if (num_inliers_ < min_inliers_) {
    cout << "reject because inlier is too small: " << num_inliers_ << endl;
    return false;
  }
  // // if the motion is too large, it is probably wrong  T的模太大，false
  // // 由李群求李代数，在李代数上面是可以比较大小的，使用对数获取变换矩阵的李代数(6维)
  // Sophus::Vector6d d = T_c_r_estimated_.log();
  // if the motion is too large, it is probably wrong
  // ref表示上一帧，即Trw*Twc=Trc，即上一帧到当前帧的T
  SE3 T_r_c = ref_->T_c_w_ * T_c_w_estimated_.inverse();
  //求se3李代数
  Sophus::Vector6d d = T_r_c.log();
  // 如果估算的位姿太大则存在问题，返回false
  // 求李代数的范数
  if (d.norm() > 5.0) {
    cout << "reject because motion is too large: " << d.norm() << endl;
    return false;
  }
  return true;
}

// // 检查关键帧，T中R或t比较筛选关键帧
// bool VisualOdometry::checkKeyFrame() {Sophus::Vector6d d = T_c_r_estimated_.log();

// ref表示上一帧，即Trw*Twc=Trc，即上一帧到当前帧的T
bool VisualOdometry::checkKeyFrame() { SE3 T_r_c = ref_->T_c_w_ * T_c_w_estimated_.inverse();
  //求se3李代数
  Sophus::Vector6d d = T_r_c.log();

  Vector3d trans = d.head<3>();
  Vector3d rot = d.tail<3>();
  //在添加关键帧到map中之前，检测关键帧，如果当前旋转和平移都大于最小值则返回true，否则返回false
  // key_frame_min_rot和key_frame_min_trans在vo构造函数中被赋值
  //即只要旋转或者平移超过一定距离就可以被认为是关键帧
  if (rot.norm() > key_frame_min_rot || trans.norm() > key_frame_min_trans)
    return true;
  //返回false表示当前位姿变化不合理，传入帧将不加入map中作为关键帧
  return false;
}
// 新增，用于增加关键帧，第一帧就是关键帧
// 其中的map_成员变量在构造函数中被复制，一开始应该是空的
// 添加关键帧函数（将关键帧特征点加入map匹配点集合）
// void VisualOdometry::addKeyFrame() {
//   cout << "adding a key-frame" << endl;
//   map_->insertKeyFrame(curr_);
// }

void VisualOdometry::addKeyFrame() {
  // keyframes_为map容器类对象，empty（）方法返回容器是否为空
  //第一帧的时候肯定是空的，在提取第一帧的特征点之后，将第一帧的所有特征点放入地图中
  if (map_->keyframes_.empty()) {
    // first key-frame, add all 3d points into map
    //在使用之前，已经检测出来了keypoints_curr_
    for (size_t i = 0; i < keypoints_curr_.size(); i++) {
      // curr帧对象已经包含了depth_，即深度图像矩阵
      //算出当前帧的关键点的深度
      double d = curr_->findDepth(keypoints_curr_[i]);
      if (d < 0) continue;
      //在第一帧的时候ref=curr=pframe，其中pframe包含了相机参数
      //一开始的Tcw会自动变成单位矩阵
      Vector3d p_world = ref_->camera_->pixel2world(
          Vector2d(keypoints_curr_[i].pt.x, keypoints_curr_[i].pt.y),
          curr_->T_c_w_, d);
      //好象是参考帧的相机中心到当前的关键点对应的三维点的位移
      Vector3d n = p_world - ref_->getCamCenter();
      //归一化，单位化
      n.normalize();
      // createMapPoint函数返回托管Mappoint类的对象的指针
      //这个用于将第一帧的点坐标，描述子，frame等信息传给MapPoint类，保存在这里
      // curr_.get()，智能指针的get() 返回curr托管的Frame类的指针
      MapPoint::Ptr map_point = MapPoint::createMapPoint(
          p_world, n, descriptors_curr_.row(i).clone(), curr_.get());
      //将包含信息的map_point插入容器中
      //在insertMapPoint这里会输出关键帧的个数
      map_->insertMapPoint(map_point);
    }
  }
  // map有两个功能，管理地图中的点，和关键帧
  //如果不是第一帧直接执行这里
  map_->insertKeyFrame(curr_);
  ref_ = curr_;
}
//新增函数，增加地图中的点。局部地图类似于slidewindow一样，随时的增删地图中的点，来跟随运动
//后面如果发现地图中的点的数量不够，会调用它
void VisualOdometry::addMapPoints() {
  // add the new map points into map
  //创建一个bool型的数组matched，大小为当前keypoints数组大小，值全为false
  vector<bool> matched(keypoints_curr_.size(), false);
  //首先这个match_2dkp_index_是新来的当前帧跟地图匹配时，好的匹配到的关键点在keypoins数组中的索引
  //在这里将已经匹配的keypoint索引置为true，因为之后增加的肯定是之前没匹配的
  for (int index : match_2dkp_index_) matched[index] = true;
  //遍历当前keypoints数组，然后将深度大于0的关键点都加入地图中
  for (int i = 0; i < keypoints_curr_.size(); i++) {
    //如果为true，说明在地图中找到了匹配，也就意味着地图中已经有这个点了。直接continue
    if (matched[i] == true) continue;
    double d = ref_->findDepth(keypoints_curr_[i]);
    if (d < 0) continue;
    //如果没有continue的话，说明这个点在地图中没有找到匹配，认定为新的点，
    //下一步就是找到depth数据，构造3D点，然后构造地图点，添加进地图即可。
    Vector3d p_world = ref_->camera_->pixel2world(
        Vector2d(keypoints_curr_[i].pt.x, keypoints_curr_[i].pt.y),
        curr_->T_c_w_, d);
    Vector3d n = p_world - ref_->getCamCenter();
    n.normalize();
    MapPoint::Ptr map_point = MapPoint::createMapPoint(
        p_world, n, descriptors_curr_.row(i).clone(), curr_.get());
    //将地图中原来没有的点添加进地图
    map_->insertMapPoint(map_point);
  }
}

//新增函数：优化地图。主要是为了维护地图的规模。删除一些地图点，在点过少时增加地图点等操作。
void VisualOdometry::optimizeMap() {
  // remove the hardly seen and no visible points
  //利用auto自动为变量选取类型，在for循环的时候适合使用auto
  for (auto iter = map_->map_points_.begin();
       iter != map_->map_points_.end();) {
    // iter->second表示map容器中元素的值vlaue，由于值为mappoint类指针，因此还可以指向类的成员变量pos_
    //判断世界坐标系的点是否在当前good_画面的视野中
    //如果点在当前帧都不可见了，说明跑的比较远了，删掉
    if (!curr_->isInFrame(iter->second->pos_)) {
      //删除掉iter指向的这个元素，返回下一个元素的迭代器
      iter = map_->map_points_.erase(iter);
      continue;
    }
    //定义匹配率，用匹配次数/可见次数，匹配率过低说明经常见但是没有几次匹配。
    //应该是一些比较难识别的点，也就是出来的描述子比价奇葩。所以删掉
    float match_ratio =
        float(iter->second->matched_times_) / iter->second->visible_times_;
    if (match_ratio < map_point_erase_ratio_) {
      iter = map_->map_points_.erase(iter);
      continue;
    }
    //得到当前帧和地图点之间的夹角，角度过大，删除
    double angle = getViewAngle(curr_, iter->second);
    if (angle > M_PI / 6.) {
      iter = map_->map_points_.erase(iter);
      continue;
    }
    //继续，可以根据一些其他条件自己添加要删除点的情况
    if (iter->second->good_ == false) {
      // TODO try triangulate this map point
    }
    iter++;
  }
  //下面说一些增加点的情况，首先当前帧去地图中匹配时，点少于100个了，
  // 一般情况是运动幅度过大了，跟之前的帧没多少交集了，所以增加一下。
  if (match_2dkp_index_.size() < 100) addMapPoints();
  //如果点过多了，多于1000个，适当增加释放率，让地图处于释放点的趋势。
  if (map_->map_points_.size() > 1000) {
    // TODO map is too large, remove some one
    map_point_erase_ratio_ += 0.05;
  }
  //如果没有多于1000个，保持释放率在0.1，维持地图的体量为平衡态
  else
    map_point_erase_ratio_ = 0.1;
  cout << "map points: " << map_->map_points_.size() << endl;
}
//取得一个空间点在一个帧下的视角角度。返回值是double类型的角度值。
double VisualOdometry::getViewAngle(Frame::Ptr frame, MapPoint::Ptr point) {
  //构造发方法是空间点坐标减去相机中心坐标。得到从相机中心指向指向空间点的向量。
  Vector3d n = point->pos_ - frame->getCamCenter();
  n.normalize();  //单位化
  //返回一个角度，acos()为反余弦，
  //向量*乘为：a*b=|a||b|cos<a,b>
  //所以单位向量的*乘得到的是两个向量的余弦值，再用acos()即得到角度，返回
  //物理意义就是得到匆匆世界坐标系下看空间点和从相机坐标系下看空间点，视角差的角度。
  // norm表示相机的朝向
  return acos(n.transpose() * point->norm_);
}

}  // namespace myslam