#include <boost/timer.hpp>
#include <fstream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/viz.hpp>

#include "myslam/config.h"
#include "myslam/visual_odometry.h"

int main(int argc, char **argv) {
  // 读取参数文件
  // myslam::Config::setParameterFile(argc[1]
  // "/home/q/CLionProjects/8-slam/project/0.1/config/default.yaml");
  myslam::Config::setParameterFile(argv[1]);
  // 实例化 vo new myslam::VisualOdometry 的是 VisualOdometry 的构造函数
  // 实例化vo对象
  myslam::VisualOdometry::Ptr vo(new myslam::VisualOdometry);
  // 从文件中读取数据的路径
  string dataset_dir = myslam::Config::get<string>("dataset_dir");
  cout << "dataset: " << dataset_dir << endl;
  // 从文件中读取文件 associate.txt名字和时间戳
  ifstream fin(dataset_dir + "/associate.txt");
  if (!fin) {
    cout << "please generate the associate file called associate.txt!" << endl;
    return 1;
  }
  // 创建存储彩色图和深度图像的字符串容器
  vector<string> rgb_files, depth_files;
  // 创建存储彩色图和深度图像的时间戳的双精度数容器
  vector<double> rgb_times, depth_times;
  // 循环读取文件中的数据，并且用逗号为分割符存储到rgb_time, rgb_file,
  // depth_time, depth_file这四个容器中 文件格式：1305031453.359684
  // rgb/1305031453.359684.png 1305031453.374112 depth/1305031453.374112.png
  while (!fin.eof()) {
    string rgb_time, rgb_file, depth_time, depth_file;
    fin >> rgb_time >> rgb_file >> depth_time >> depth_file;
    // atof 将字符串数据类型转换为浮点数类型 头文件为<stdlib.h>
    rgb_times.push_back(atof(rgb_time.c_str()));
    depth_times.push_back(atof(depth_time.c_str()));
    // + 图片数据完整路径拼接
    rgb_files.push_back(dataset_dir + "/" + rgb_file);
    depth_files.push_back(dataset_dir + "/" + depth_file);
    // fin.good是判断文件是否打开的，如果返回真的话就是打开了，否则没有打开
    if (fin.good() == false) break;
  }

  // myslam::Camera::Ptr 的方法 new myslam::Camera 实例化一个 camera
  // 智能指针调用构造函数创建一个对象
  myslam::Camera::Ptr camera(new myslam::Camera);
  // 创建一个可视化空窗口,窗口名称为“Visual Odometry” visualization
  cv::viz::Viz3d vis("Visual Odometry");
  // 创建world_coor　camera_coor两个坐标系，坐标系是以Widget类型存在的
  cv::viz::WCoordinateSystem world_coor(1.0), camera_coor(0.5);
  // 构造makeCameraPose()函数需要三个三维点确定三个轴的方向的三个向量
  // 蓝色-Z，红色-X，绿色-Y
  // 设置视角有利于观察，不设置会有默认视角，可能会比较别扭。开始后拖动鼠标可以改变观察视角。
  // Affine3d3D位姿参数 利用罗德里格斯公式将较为直观的旋转向量转换为旋转矩阵
  // makeCameraPose()构造相机在世界坐标系下位姿
  // cam_pos 相机的世界坐标，
  // cam_focal_point 相机中心点坐标，
  // 相机y轴朝向：cam_y_dir
  // 用makeCameraPose()函数构造Affine3d类型的相机位姿，这里其实是视角位姿，也就是程序开始时你处在什么视角看
  cv::Point3d cam_pos(0, -1.0, -1.0), cam_focal_point(0, 0, 0),
      cam_y_dir(0, 1, 0);
  cv::Affine3d cam_pose =
      cv::viz::makeCameraPose(cam_pos, cam_focal_point, cam_y_dir);
  // 用setViewerPose()设置观看视角
  vis.setViewerPose(cam_pose);
  // setRenderingProperty()函数设置渲染显示属性，第一个参数是个枚举，对应要渲染的属性是线宽，后面是属性值
  world_coor.setRenderingProperty(cv::viz::LINE_WIDTH, 2.0);
  camera_coor.setRenderingProperty(cv::viz::LINE_WIDTH, 1.0);
  // 用showWidget()函数将部件添加到窗口内
  vis.showWidget("World", world_coor);
  vis.showWidget("Camera", camera_coor);
  // 至此，窗口中已经显示了全部需要显示的东西，两个坐标系：世界坐标系，相机坐标系。
  // 世界坐标系就是不动的，要做的就是计算出各个帧的相机坐标系的位置
  // 下面的for循环，不断的给相机坐标系设置新的pose，然后达到动画的效果。输出RGB图像信息，共读到文件数
  cout << "read total " << rgb_files.size() << " entries" << endl;
  // 整个画面的快速刷新呈现动态，由for循环控制
  for (int i = 0; i < rgb_files.size(); i++) {
    cout << "****** loop " << i << " ******" << endl;
    // 读取图像 创建帧操作 这里传入彩色图和深度图
    // 使用的是pnp的方法计算参考帧和当前帧之间的位置姿态
    Mat color = cv::imread(rgb_files[i]);
    Mat depth = cv::imread(depth_files[i], -1);
    if (color.data == nullptr || depth.data == nullptr) break;
    // 实例化一个Frame对象
    // 创建新的一帧（包括图片信息，相机内参和时间戳），随后调用addFrame进行计算
    myslam::Frame::Ptr pFrame = myslam::Frame::createFrame();
    // 为对象中的变量赋值 往新的一帧中添加数据
    pFrame->camera_ = camera;
    pFrame->color_ = color;
    pFrame->depth_ = depth;
    pFrame->time_stamp_ = rgb_times[i];

    // 记录开始时间
    boost::timer timer;
    // 向 vo 中加入Frame，并计算先后提取特征点，计算描述子，特征点匹配等
    // 这里将帧添加进去，进行位姿变换计算
    vo->addFrame(pFrame);
    // 计算 vo 消耗的时间
    cout << "VO costs time: " << timer.elapsed() << endl;
    if (vo->state_ == myslam::VisualOdometry::LOST) break;
    SE3 Twc = pFrame->T_c_w_.inverse();

    // 如果 vo 的状态量变成 “LOST” 则程序终止。
    if (vo->state_ == myslam::VisualOdometry::LOST) break;
    // 可视化窗口中动的是相机坐标系,所以本质上是求取相机坐标系下的点在世界坐标系下的坐标,Pw=Twc*Pc;
    SE3 Tcw = pFrame->T_c_w_.inverse();
    // show the map and the camera pose
    // 用Twc构造Affine3d类型的pose所需要的旋转矩阵和平移矩阵

    // cv::Affine3d M(
    //     cv::Affine3d::Mat3(
    //         Tcw.rotation_matrix()(0, 0), Tcw.rotation_matrix()(0, 1),
    //         Tcw.rotation_matrix()(0, 2), Tcw.rotation_matrix()(1, 0),
    //         Tcw.rotation_matrix()(1, 1), Tcw.rotation_matrix()(1, 2),
    //         Tcw.rotation_matrix()(2, 0), Tcw.rotation_matrix()(2, 1),
    //         Tcw.rotation_matrix()(2, 2)),
    //     cv::Affine3d::Vec3(Tcw.translation()(0, 0), Tcw.translation()(1, 0),
    //                        Tcw.translation()(2, 0)));
    // //两窗口同时显示，一个是图像
    // cv::imshow("image", color);



    
   // show the map and the camera pose
    cv::Affine3d M(
        cv::Affine3d::Mat3(
            Twc.rotation_matrix()(0, 0), Twc.rotation_matrix()(0, 1),
            Twc.rotation_matrix()(0, 2), Twc.rotation_matrix()(1, 0),
            Twc.rotation_matrix()(1, 1), Twc.rotation_matrix()(1, 2),
            Twc.rotation_matrix()(2, 0), Twc.rotation_matrix()(2, 1),
            Twc.rotation_matrix()(2, 2)),
        cv::Affine3d::Vec3(Twc.translation()(0, 0), Twc.translation()(1, 0),Twc.translation()(2, 0)));
    // 在图像上圈出特征点
    Mat img_show = color.clone();           
    //遍历地图上的所有的地图点
    for (auto& pt : vo->map_->map_points_)  
    {
      myslam::MapPoint::Ptr p = pt.second;
      Vector2d pixel = pFrame->camera_->world2pixel(p->pos_, pFrame->T_c_w_);
      //画圆
      cv::circle(img_show, cv::Point2f(pixel(0, 0), pixel(1, 0)), 5,cv::Scalar(0, 255, 0), 2);
    }

    cv::imshow("image", img_show);
    cv::waitKey(1);
    // 另外一个就是viz可视化窗口
    // 就是一个个帧的位姿，然后是把第一帧当做世界坐标系，然后转换成世界坐标系周后的位姿，然后在窗口中显示。
    vis.setWidgetPose("Camera", M);
    vis.spinOnce(1, false);
  }

  return 0;
}
