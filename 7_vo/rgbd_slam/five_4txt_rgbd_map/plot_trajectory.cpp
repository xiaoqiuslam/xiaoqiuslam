#include <pangolin/pangolin.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <unistd.h>

int main(int argc, char **argv) {
    std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> poses;
    // path to trajectory file
    std::ifstream fin("../pose.txt");
    if (!fin) {
        std::cout << "cannot find trajectory file" << std::endl;
        return 1;
    }
    while (!fin.eof()) {
        // 四元数和平移向量一起组装成变换矩阵
        double tx, ty, tz, qx, qy, qz, qw;
        fin >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
        // 欧氏变换矩阵使用 Eigen::Isometry 虽然称为3d,实质上是4＊4的矩阵
        // 旋转矩阵代表机器人运动轨迹的朝向,平移部分代表机器人运动轨迹的移动,移动向量是连续的,构成了机器人的移动轨迹。
        // Twc 代表的是机器人相机视角观察世界坐标系, 就是机器人在世界坐标系下面的坐标。
        // 世界坐标系是不动的,在笛卡尔坐标系中可以认为 Twc 是机器人在相对原点坐标在移动,当这个移动可视化呈现在观察者眼中,即是机器人的运动轨迹。
        Eigen::Isometry3d Twr(Eigen::Quaterniond(qw, qx, qy, qz));
        Twr.pretranslate(Eigen::Vector3d(tx, ty, tz));
        poses.push_back(Twr);
    }
    std::cout << "read total " << poses.size() << " pose entries" << std::endl;

    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("trajectory viewer", 1024, 640);
    //　启动深度测试
    glEnable(GL_DEPTH_TEST);
    //　启用色彩混合
    glEnable(GL_BLEND);
    // 设置颜色混合 透明度叠加计算,GL_SRC_ALPHA：表示使用源颜色的alpha值来作为因子,GL_ONE_MINUS_SRC_ALPHA：表示用1.0减去源颜色的alpha值来作为因子
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // 设置相机参数 pangolin::ProjectionMatrix
    // OpenGlMatrixSpec ProjectionMatrix(int w, int h, double fu, double fv, double u0, double v0, double zNear, double zFar )
    // return ProjectionMatrixRUB_BottomLeft(w,h,fu,fv,u0,v0,zNear,zFar);

    //OpenGlMatrix ModelViewLookAt(double ex, double ey, double ez, double lx, double ly, double lz, AxisDirection up)
    //  const double* u = AxisDirectionVector[up];
    //  return ModelViewLookAt(ex,ey,ez,lx,ly,lz,u[0],u[1],u[2]);


    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));

    // 定义面板前两个参数（0.0, 1.0）表明宽度和面板纵向宽度和窗口大小相同
    // 中间两个参数（pangolin::Attach::Pix(175), 1.0）表明右边所有部分用于显示图形
    // 最后一个参数（-1024.0f/768.0f）为显示长宽比
    // 清除屏幕并激活要渲染到的视图
    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        glLineWidth(2);
        for (size_t i = 0; i < poses.size(); i++) {
            // 画每个位姿的三个坐标轴,坐标系的原点实际上就是读取每一个位姿点的平移向量,然后以平移向量为原点,构建每个位姿的坐标系
            Eigen::Vector3d Ow = poses[i].translation();
            // std::cout << "Ow\n" << Ow << std::endl;
            // 点(1,0,0)左乘上变换矩阵,进行坐标变换得到坐标系，然后做两点之间的连线,得到轨迹
            // 当前位姿平移向量x方向的分量作为x轴
            Eigen::Vector3d Xw = poses[i] * (0.03 * Eigen::Vector3d(1, 0, 0));
            // cout << "Xw\n"<< Xw << endl;
            // 当前位姿平移向量y方向的分量作为y轴
            Eigen::Vector3d Yw = poses[i] * (0.03 * Eigen::Vector3d(0, 1, 0));
            // cout << "Yw\n"<< Yw << endl;
            // 当前位姿平移向量y方向的分量作为y轴
            Eigen::Vector3d Zw = poses[i] * (0.03 * Eigen::Vector3d(0, 0, 1));
            // cout << "Zw\n"<< Zw << endl;
            glBegin(GL_LINES);
            // 红色 x 轴 向量　Ow Xw
            glColor3f(1.0, 0.0, 0.0);
            glVertex3d(Ow[0], Ow[1], Ow[2]);
            glVertex3d(Xw[0], Xw[1], Xw[2]);
            // 绿色 y 轴 向量 Ow Yw
            glColor3f(0.0, 1.0, 0.0);
            glVertex3d(Ow[0], Ow[1], Ow[2]);
            glVertex3d(Yw[0], Yw[1], Yw[2]);

            // 蓝色 z 轴 向量 Ow Zw
            glColor3f(0.0, 0.0, 1.0);
            glVertex3d(Ow[0], Ow[1], Ow[2]);
            glVertex3d(Zw[0], Zw[1], Zw[2]);
            glEnd();
        }
//        // 画出连线
//        for (size_t i = 0; i < poses.size(); i++) {
//            // 两个位姿之间连线的颜色 黑色
//            glColor3f(0.0, 0.0, 0.0);
//            glBegin(GL_LINES);
//            auto p1 = poses[i], p2 = poses[i + 1];
//            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
//            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
//            glEnd();
//        }
        pangolin::FinishFrame();
        // sleep
        usleep(5000);
    }
    return 0;
}
