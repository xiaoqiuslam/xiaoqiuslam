#include "System.h"
#include <thread>

namespace ORB_SLAM2 {
    System::System(const string &strSettingsFile){
        cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
        mpTracker = new Tracking(this, strSettingsFile);
    }

    cv::Mat System::TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight) {
        cv::Mat Tcw = mpTracker->GrabImageStereo(imLeft, imRight);
        return Tcw;
    }
}  // namespace ORB_SLAM2
