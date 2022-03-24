## 目录contents

a graph of code libraries

* [00base](00base)

##### step1

* [01common](01common )

##### step2

* [02kdtree k维tree](02kdtree)
* [03octree 八叉树](03octree)
* [04search](04search)
* [05sample consensus  抽样一致性模块](05sampleconsensus抽样一致性模块)
* [06range-images深度图像](06range-images深度图像)
* [07 ...]()

##### step3

* [08 io 输入输出](08IO输入输出)
* [09 filters 滤波](09filters滤波)
* [10 features 特征](10features特征)

##### step4

* [11 surface表面 ](11surface表面 )
* [12 segmentation分割](12segmentation分割)
* [13 recognition识别](13recognition识别)
* [14 registration配准](14registration配准)
* [15 visualization可视化](15visualization可视化)
* [16 keypoints关键点](16keypoints关键点)
* [17tracking](17tracking )


PCL完全是一个的模块化的现代C++模板库。其基于以下第三方库：**Boost、Eigen、FLANN、VTK、CUDA、OpenNI、Qhull**，
实现点云相关的**获取、滤波、分割、配准、检索、特征提取、识别、追踪、曲面重建、可视化等。**

- [01common](https://github.com/HuangCongQing/pcl-learning/blob/master/01common)
- [02kdtree k维tree](https://github.com/HuangCongQing/pcl-learning/blob/master/02kdtree)
- [03octree 八叉树](https://github.com/HuangCongQing/pcl-learning/blob/master/03octree)
- [04search](https://github.com/HuangCongQing/pcl-learning/blob/master/04search)
- [05sample consensus 抽样一致性模块](https://github.com/HuangCongQing/pcl-learning/blob/master/05sampleconsensus%E6%8A%BD%E6%A0%B7%E4%B8%80%E8%87%B4%E6%80%A7%E6%A8%A1%E5%9D%97)
- [06range-images深度图像](https://github.com/HuangCongQing/pcl-learning/blob/master/06range-images%E6%B7%B1%E5%BA%A6%E5%9B%BE%E5%83%8F)
- [07tracking](https://github.com/HuangCongQing/pcl-learning/blob/master/17tracking) （此模块，没有官方示例代码）
- [08 io 输入输出](https://github.com/HuangCongQing/pcl-learning/blob/master/08IO%E8%BE%93%E5%85%A5%E8%BE%93%E5%87%BA)
- [09 filters 滤波](https://github.com/HuangCongQing/pcl-learning/blob/master/09filters%E6%BB%A4%E6%B3%A2)
- [10 features 特征](https://github.com/HuangCongQing/pcl-learning/blob/master/10features%E7%89%B9%E5%BE%81)
- [11 surface表面](https://github.com/HuangCongQing/pcl-learning/blob/master/11surface%E8%A1%A8%E9%9D%A2)
- [12 segmentation分割](https://github.com/HuangCongQing/pcl-learning/blob/master/12segmentation%E5%88%86%E5%89%B2)
- [13 recognition识别](https://github.com/HuangCongQing/pcl-learning/blob/master/13recognition%E8%AF%86%E5%88%AB)（下图中没有）
- [14 registration配准](https://github.com/HuangCongQing/pcl-learning/blob/master/14registration%E9%85%8D%E5%87%86)
- [15 visualization可视化](https://github.com/HuangCongQing/pcl-learning/blob/master/15visualization%E5%8F%AF%E8%A7%86%E5%8C%96)
- [16 keypoints关键点](https://github.com/HuangCongQing/pcl-learning/blob/master/16keypoints%E5%85%B3%E9%94%AE%E7%82%B9)


- [01-点云中的数学](http://robot.czxy.com/docs/pcl/chapter03/point_cloud_math/)
- [02-点云配准原理概述](http://robot.czxy.com/docs/pcl/chapter03/registration_intro/)
- [03-点云配准流程示例 *](http://robot.czxy.com/docs/pcl/chapter03/registration/)
- [使用迭代最近点算法(ICP)](http://robot.czxy.com/docs/pcl/chapter03/registration/#icp)
- [正态分布变换配准(NDT)](http://robot.czxy.com/docs/pcl/chapter03/registration/#ndt)
- [刚性物体的鲁棒姿态估计](http://robot.czxy.com/docs/pcl/chapter03/registration/#_9)
- [04-配准之交互式ICP](http://robot.czxy.com/docs/pcl/chapter03/registration_interactive_icp/)
- [05-点云配准数学原理](http://robot.czxy.com/docs/pcl/chapter03/registration_theory/)
- [PCL 可视化](https://www.cnblogs.com/li-yao7758258/p/6442156.html)
- [PCLVisualizer可视化类](https://www.cnblogs.com/li-yao7758258/p/6445127.html)
- [可视化深度图像](https://www.cnblogs.com/li-yao7758258/p/6444207.html)


## 编译过程

```shell
mkdir build
cd build
cmake .. // 对上一级进行编译
make  // 生成可执行文件命令
./executedemo  // 运行可执行文件
```
