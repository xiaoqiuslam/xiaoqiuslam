#ifndef SYSTEM_H
#define SYSTEM_H

#include <unistd.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <string>
#include <thread>

#include "Tracking.h"

using namespace std;

namespace ORB_SLAM2 {

class Tracking;

class System {
 public:
  // Initialize the SLAM system. It launches the Local Mapping, Loop Closing and
  System(const string &strSettingsFile);

  // Proccess the given stereo frame. Images must be synchronized and rectified.
  // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to
  // grayscale. Returns the camera pose (empty if tracking fails).
  cv::Mat TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight);


 private:
  // Tracker. It receives a frame and computes the associated camera pose.
  // It also decides when to insert a new keyframe, create some new MapPoints
  // and performs relocalization if tracking fails.
  Tracking *mpTracker;

};
}  // namespace ORB_SLAM2

#endif  // SYSTEM_H