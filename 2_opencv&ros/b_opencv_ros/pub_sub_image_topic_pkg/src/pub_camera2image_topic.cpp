#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <stdio.h>
 
int main(int argc, char** argv)
{
    // 通过终端传参方式更改相机ID
    if(argv[1] == NULL)   
    {  
        ROS_INFO("argv[1]=NULL\n");  
        return 1;  
    }  
    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("camera/image", 1);

    // 通过终端传参方式更改相机ID
    std::istringstream video_sourceCmd(argv[1]); 
    int video_source; 

    if(!(video_sourceCmd >> video_source))   
    {  
        ROS_INFO("video_sourceCmd is %d\n",video_source);  
        return 1;  
    }  
    
    
    // video_source 代表的数字为要索引的摄像头
    cv::VideoCapture cap(video_source);
    if(!cap.isOpened()) 
    {
        ROS_INFO("can not opencv video device\n");
        return 1;
    }
    cv::Mat frame;
    sensor_msgs::ImagePtr msg;
    // 以10ms间隔发送图片
    ros::Rate loop_rate(10);
    while (nh.ok()) 
    {
        // 从摄像头抓取一帧图像
        cap >> frame;
        if (!frame.empty()) {  
            msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();  
            pub.publish(msg);  
        }
        ROS_INFO("runnning!");
        ros::spinOnce();  
        // 与ros::Rate loop_rate相对应,休息10ms
        loop_rate.sleep();
    }
    
    ros::spin();
}


