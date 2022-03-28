#ifndef ORBEXTRACTOR_H
#define ORBEXTRACTOR_H

#include <opencv/cv.h>

#include <list>
#include <vector>
#include <string>



namespace ORB_SLAM2 {

class ExtractorNode {
 public:
  ExtractorNode() : bNoMore(false) {}

  void DivideNode(ExtractorNode &n1, ExtractorNode &n2, ExtractorNode &n3,
                  ExtractorNode &n4);

  std::vector<cv::KeyPoint> vKeys;
  cv::Point2i UL, UR, BL, BR;
  std::list<ExtractorNode>::iterator lit;
  bool bNoMore;
};

class ORBextractor {
 public:
  enum { HARRIS_SCORE = 0, FAST_SCORE = 1 };

  ORBextractor(int nfeatures, float scaleFactor, int nlevels, int iniThFAST,
               int minThFAST);
  // 在FAST提取角点进行分块后有可能在某个块中在iniThFAST原始阈值情况下提取不到角点使用更小的minThFAST阈值进一步提取

  ~ORBextractor() {}

  // Compute the ORB features and descriptors on an image.
  // ORB are dispersed on the image using an octree.
  // Mask is ignored in the current implementation.
  int operator()(cv::InputArray _image, cv::InputArray _mask,
                 std::vector<cv::KeyPoint> &_keypoints,
                 cv::OutputArray _descriptors, std::vector<int> &vLappingArea);

  int inline GetLevels() { return nlevels; }

  float inline GetScaleFactor() { return scaleFactor; }

  std::vector<float> inline GetScaleFactors() { return mvScaleFactor; }

  std::vector<float> inline GetInverseScaleFactors() {
    return mvInvScaleFactor;
  }
  // 存储缩放系数平方的vector
  std::vector<float> inline GetScaleSigmaSquares() { return mvLevelSigma2; }

  std::vector<float> inline GetInverseScaleSigmaSquares() { return mvInvLevelSigma2; }

  std::vector<cv::Mat> mvImagePyramid;

 protected:
  void ComputePyramid(cv::Mat image);
  // 利用四叉树提取高斯金字塔中每层图像的orb关键点
  void ComputeKeyPointsOctTree(
      std::vector<std::vector<cv::KeyPoint> > &allKeypoints);
  std::vector<cv::KeyPoint> DistributeOctTree(
      // 将关键点分配到四叉树,筛选关键点
      const std::vector<cv::KeyPoint> &vToDistributeKeys, const int &minX,
      const int &maxX, const int &minY, const int &maxY, const int &nFeatures,
      const int &level);

  std::vector<cv::Point> pattern;

  int nfeatures;
  double scaleFactor;
  int nlevels;
  int iniThFAST;
  int minThFAST;

  std::vector<int> mnFeaturesPerLevel;

  // Patch圆的u轴方向最大坐标
  std::vector<int> umax;

  std::vector<float> mvScaleFactor;
  std::vector<float> mvInvScaleFactor;
  std::vector<float> mvLevelSigma2;
  std::vector<float> mvInvLevelSigma2;
};

}  // namespace ORB_SLAM2

#endif
