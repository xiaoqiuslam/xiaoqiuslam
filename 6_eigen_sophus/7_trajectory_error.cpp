#include <iostream>
#include <unistd.h>
#include <pangolin/pangolin.h>
#include <sophus/se3.h>

typedef std::vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> TrajectoryType;

void DrawTrajectory(const TrajectoryType &gt, const TrajectoryType &esti);

TrajectoryType ReadTrajectory(const std::string &path);

int main(int argc, char **argv) {
  TrajectoryType groundtruth = ReadTrajectory("../groundtruth.txt");
  TrajectoryType estimated = ReadTrajectory("../estimated.txt");
  assert(!groundtruth.empty() && !estimated.empty());
  assert(groundtruth.size() == estimated.size());

  // compute rmse 位姿的均方根误差
  double rmse = 0;
  for (size_t i = 0; i < estimated.size(); i++) {
    Sophus::SE3 p1 = estimated[i], p2 = groundtruth[i];
    // 李群SE3的对数映射求李代数se3
    // 对应视觉SLAM十四讲第二版p89 公式4.44
    // 计算向量的二范数
    double error = (p2.inverse() * p1).log().norm();
    rmse += error * error;
  }
  rmse = rmse / double(estimated.size());
  rmse = sqrt(rmse);
  std::cout << "RMSE = " << rmse << std::endl;

  DrawTrajectory(groundtruth, estimated);
  return 0;
}

TrajectoryType ReadTrajectory(const std::string &path) {
    std::ifstream fin(path);
  TrajectoryType trajectory;
  if (!fin) {
      std::cerr << "trajectory " << path << " not found." << std::endl;
    return trajectory;
  }

  while (!fin.eof()) {
    double time, tx, ty, tz, qx, qy, qz, qw;
    // tx  ty  tz 为Twc的平移部分
    // qx  qy  qz  qw 是四元数表示的 Twc的旋转部分 qw 是四元数的实部
    fin >> time >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
    // 四元数和平移向量构造李群SE3变换矩阵
    Sophus::SE3 p1(Eigen::Quaterniond(qw, qx, qy, qz), Eigen::Vector3d(tx, ty, tz));
    trajectory.push_back(p1);
  }
  return trajectory;
}

void DrawTrajectory(const TrajectoryType &gt, const TrajectoryType &esti) {
  // create pangolin window and plot the trajectory
  pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  pangolin::OpenGlRenderState s_cam(
      pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
      pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
  );

  pangolin::View &d_cam = pangolin::CreateDisplay()
      .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
      .SetHandler(new pangolin::Handler3D(s_cam));


  while (pangolin::ShouldQuit() == false) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    d_cam.Activate(s_cam);
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

    glLineWidth(2);
    for (size_t i = 0; i < gt.size() - 1; i++) {
      glColor3f(0.0f, 0.0f, 1.0f);  // blue for ground truth
      glBegin(GL_LINES);
      // 轨迹就是平移向量
      auto p1 = gt[i], p2 = gt[i + 1];
      glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
      glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
      glEnd();
    }

    for (size_t i = 0; i < esti.size() - 1; i++) {
      glColor3f(1.0f, 0.0f, 0.0f);  // red for estimated
      glBegin(GL_LINES);
      auto p1 = esti[i], p2 = esti[i + 1];
      glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
      glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
      glEnd();
    }
    pangolin::FinishFrame();
    usleep(5000);
  }

}
