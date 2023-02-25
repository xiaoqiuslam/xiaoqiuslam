#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>

pcl::visualization::CloudViewer viewer("viewer");

pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud ( new pcl::PointCloud<pcl::PointXYZRGB> );

int main(int argc, char **argv) {

    double fx = 718.856, fy = 718.856, cx = 607.1928, cy = 185.2157;
    double b = 0.573;

    cv::Mat left = cv::imread("../left.png", 0);
    cv::Mat right = cv::imread("../right.png", 0);
    cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(0, 96, 9, 8 * 9 * 9, 32 * 9 * 9, 1, 63, 10, 100, 32);
    cv::Mat disparity_sgbm, disparity;
    sgbm->compute(left, right, disparity_sgbm);
    disparity_sgbm.convertTo(disparity, CV_32F, 1.0 / 16.0f);

    for (int v = 0; v < left.rows; v++)
        for (int u = 0; u < left.cols; u++) {
            if (disparity.at<float>(v, u) <= 0.0 || disparity.at<float>(v, u) >= 96.0)
                continue;

            double x = (u - cx) / fx;
            double y = (v - cy) / fy;
            double depth = fx * b / (disparity.at<float>(v, u));


            pcl::PointXYZRGB pcl_point;
            pcl_point.z = depth;
            pcl_point.x = x * depth;
            pcl_point.y = y * depth;

            pcl_point.b = left.at<uchar>(v, u) / 255.0;
            pcl_point.g = left.at<uchar>(v, u) / 255.0;
            pcl_point.r = left.at<uchar>(v, u) / 255.0;

            cloud->points.push_back(pcl_point);
        }

    cv::imshow("disparity", disparity / 96.0);
    cv::waitKey(0);

    while (!viewer.wasStopped())
    {
        viewer.showCloud( cloud );
    }
    return 0;
}
