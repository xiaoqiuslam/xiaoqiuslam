/****************************
 * 给定一组世界坐标系下的3D点(p3d.txt)以及它在相机中对应的坐标(p2d.txt)，以及相机的内参矩阵。
 * 使用bundle adjustment 方法（g2o库实现）来估计相机的位姿T。初始位姿T为单位矩阵。
****************************/

#include <vector>
#include <fstream>
#include <iostream>
#include <opencv2/core/core.hpp>


#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

using namespace Eigen;

using namespace cv;
using namespace std;


string p3d_file = "./p3d.txt";
string p2d_file = "./p2d.txt";

void bundleAdjustment (
        const vector<Point3f> points_3d,
        const vector<Point2f> points_2d,
        Mat& K );

int main(int argc, char **argv) {


    vector< Point3f > p3d;
    vector< Point2f > p2d;

    Mat K = ( Mat_<double> ( 3,3 ) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1 );

    // 导入3D点和对应的2D点

    ifstream fp3d(p3d_file);
    if (!fp3d){
        cout<< "No p3d.text file" << endl;
        return -1;
    }
    else {
        while (!fp3d.eof()){
            double pt3[3] = {0};
            for (auto &p:pt3) {
                fp3d >> p;
            }
            p3d.push_back(Point3f(pt3[0],pt3[1],pt3[2]));
        }
    }
    ifstream fp2d(p2d_file);
    if (!fp2d){
        cout<< "No p2d.text file" << endl;
        return -1;
    }
    else {
        while (!fp2d.eof()){
            double pt2[2] = {0};
            for (auto &p:pt2) {
                fp2d >> p;
            }
            Point2f p2(pt2[0],pt2[1]);
            p2d.push_back(p2);
        }
    }

    assert(p3d.size() == p2d.size());

    int iterations = 100;
    double cost = 0, lastCost = 0;
    int nPoints = p3d.size();
    cout << "points: " << nPoints << endl;

    bundleAdjustment ( p3d, p2d, K );
    return 0;
}

void bundleAdjustment (
        const vector< Point3f > points_3d,
        const vector< Point2f > points_2d,
        Mat& K   )
{
    // creat g2o
    // new g2o version. Ref:https://www.cnblogs.com/xueyuanaichiyu/p/7921382.html

    typedef g2o::BlockSolver< g2o::BlockSolverTraits<6,3> > Block;  // pose 维度为 6, landmark 维度为 3
    // 第1步：创建一个线性求解器LinearSolver
    Block::LinearSolverType* linearSolver = new g2o::LinearSolverCSparse<Block::PoseMatrixType>();

    // 第2步：创建BlockSolver。并用上面定义的线性求解器初始化
    Block* solver_ptr = new Block (  std::unique_ptr<Block::LinearSolverType>(linearSolver) );

    // 第3步：创建总求解器solver。并从GN, LM, DogLeg 中选一个，再用上述块求解器BlockSolver初始化
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg ( std::unique_ptr<Block>(solver_ptr) );

    // 第4步：创建稀疏优化器
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm ( solver );

//    // old g2o version
//    typedef g2o::BlockSolver< g2o::BlockSolverTraits<6,3> > Block;  // pose 维度为 6, landmark 维度为 3
//    Block::LinearSolverType* linearSolver = new g2o::LinearSolverCSparse<Block::PoseMatrixType>(); // 线性方程求解器
//    Block* solver_ptr = new Block ( linearSolver );     // 矩阵块求解器
//    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg ( solver_ptr );
//    g2o::SparseOptimizer optimizer;
//    optimizer.setAlgorithm ( solver );

    // 第5步：定义图的顶点和边。并添加到SparseOptimizer中
	
	// ----------------------开始你的代码：设置并添加顶点，初始位姿为单位矩阵
 
	// ----------------------结束你的代码
	
	
    // 设置相机内参
    g2o::CameraParameters* camera = new g2o::CameraParameters (
            K.at<double> ( 0,0 ), Eigen::Vector2d ( K.at<double> ( 0,2 ), K.at<double> ( 1,2 ) ), 0);
    camera->setId ( 0 );
    optimizer.addParameter ( camera );

    // 设置边
    index = 1;
    for ( const Point2f p:points_2d )
    {
        g2o::EdgeProjectXYZ2UV* edge = new g2o::EdgeProjectXYZ2UV();
        edge->setId ( index );
        edge->setVertex ( 0, dynamic_cast<g2o::VertexSBAPointXYZ*> ( optimizer.vertex ( index ) ) );
        edge->setVertex ( 1, pose );
        edge->setMeasurement ( Eigen::Vector2d ( p.x, p.y ) );  //设置观测值
        edge->setParameterId ( 0,0 );
        edge->setInformation ( Eigen::Matrix2d::Identity() );
        optimizer.addEdge ( edge );
        index++;
    }


    // 第6步：设置优化参数，开始执行优化
    optimizer.setVerbose ( false );
    optimizer.initializeOptimization();
    optimizer.optimize ( 100 );

    // 输出优化结果
    cout<<endl<<"after optimization:"<<endl;
    cout<<"T="<<endl<<Eigen::Isometry3d ( pose->estimate() ).matrix() <<endl;
}