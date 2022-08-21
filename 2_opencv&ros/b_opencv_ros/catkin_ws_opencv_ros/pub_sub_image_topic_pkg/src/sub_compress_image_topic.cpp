#include "ros/ros.h"
#include "sensor_msgs/CompressedImage.h"
#include "sensor_msgs/image_encodings.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <iostream>
using namespace std;
cv::Mat imgCallback;
static void ImageCallback(const sensor_msgs::CompressedImageConstPtr &msg)
{
    try
    {
      cv_bridge::CvImagePtr cv_ptr_compressed = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
      imgCallback = cv_ptr_compressed->image;
      cv::imshow("imgCallback",imgCallback);
      cv::waitKey(1);
      cout<<"cv_ptr_compressed: "<<cv_ptr_compressed->image.cols<<" h: "<<cv_ptr_compressed->image.rows<<endl;
    }
    catch (cv_bridge::Exception& e)
    {
      //ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "CompressedImage");
  ros::NodeHandle nh;
  ros::Subscriber image_sub;
  std::string image_topic = "/image/compressed";
  image_sub = nh.subscribe(image_topic,10,ImageCallback);
 
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    ROS_INFO("ROS OK!");
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}