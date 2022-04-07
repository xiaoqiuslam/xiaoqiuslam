#include "System.h"
#include <thread>

namespace ORB_SLAM2 {
    System::System(const string &strSettingsFile){
        cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);

        // 4. 创建 ORB_SLAM2::Tracking 对象
        mpTracker = new Tracking(this, strSettingsFile);
    }

    cv::Mat System::TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight) {
        // 7. 执行 ORB_SLAM2::Tracking 函数 GrabImageStereo
        cv::Mat Tcw = mpTracker->GrabImageStereo(imLeft, imRight);
        return Tcw;
    }
}  // namespace ORB_SLAM2