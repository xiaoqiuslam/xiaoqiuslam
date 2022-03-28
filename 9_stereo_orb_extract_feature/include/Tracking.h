#ifndef TRACKING_H
#define TRACKING_H

#include <unistd.h>
#include <mutex>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "ORBextractor.h"
#include "System.h"
#include <opencv/highgui.h>
#include <chrono>
#include <cmath>
#include <thread>
//#include <eigen/Dense>

using namespace std;

namespace ORB_SLAM2 {
#define FRAME_GRID_ROWS 48
#define FRAME_GRID_COLS 48

class System;

class Tracking {
 public:
  Tracking(System* pSys, const string& strSettingPath);

  cv::Mat GrabImageStereo(const cv::Mat& imRectLeft, const cv::Mat& imRectRight);

  void Frame();

  static const int TH_LOW;

 public:
  enum eTrackingState {
    SYSTEM_NOT_READY = -1,
    NO_IMAGES_YET = 0,
    NOT_INITIALIZED = 1,
    OK = 2,
    LOST = 3,
    DR = 4
  };

  cv::Mat mImGray;
  cv::Mat imGrayRight;

public:

  static bool mbInitialComputations;
  cv::Mat mK;
  cv::Mat mK_r;
  cv::Mat mDistCoef;
  cv::Mat mDistCoef_r;
  cv::Mat mRrl;
  cv::Mat mtlinr;

  static float mnMinX;
  static float mnMaxX;
  static float mnMinY;
  static float mnMaxY;

  static float mfGridElementWidthInv;
  static float mfGridElementHeightInv;

  std::vector<cv::KeyPoint> mvKeys, mvKeysRight;
  std::vector<cv::KeyPoint> mvKeysUn, mvKeysRightUn;

  int N;
  int NRight;

 int monoLeft, monoRight;

  std::vector<std::size_t> mGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS];
  std::vector<std::size_t> mGridRight[FRAME_GRID_COLS][FRAME_GRID_ROWS];

  vector<float> mvLevelSigma2; //与octave对应且个数相同
  vector<float> mvInvLevelSigma2;
  std::vector<float> mvuRight;
  std::vector<float> mvDepth;
  std::vector<float> mvvRight;  //用于存放右相机匹配点的纵坐标，与mvuRight中的横坐标一一对应
  std::vector<cv::Point3f> mvP3M;
  std::vector<cv::Point3f> mvP3MRight;

  void ExtractORB(int flag, const cv::Mat &im);
  bool PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY);
  vector<size_t> GetFeaturesInAreaRight(const float &x, const float &y,
                                        const float &r, const int minLevel = -1,
                                        const int maxLevel = -1) const;


 private:

  void UndistortKeyPoints();

  void AssignFeaturesToGrid();
  void AssignFeaturesToGridRight();


  cv::Mat imLeft_cp;
  cv::Mat imRight_cp;

 protected:
  ORBextractor *mpORBextractorLeft, *mpORBextractorRight;
  cv::Mat mDescriptors, mDescriptorsRight;


  bool mbRGB;

  std::vector<cv::Point3f> mvTrackedStereoPts;
  
  std::mutex mframemutex;

  std::vector<cv::KeyPoint> mvTrackedKPs;

};

}

#endif
