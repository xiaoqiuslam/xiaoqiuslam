#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream>
#include <opencv2/opencv.hpp>
#include <iostream>
using namespace cv;
using namespace std;

int main(int argc, char** argv) {


    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("camera/image", 1);


	VideoCapture capture;
	//连接视频

	capture.open("/media/q/Q/vo/catkin_ws/src/pub_sub_image_topic_pkg/video/Mobile_Robot.mp4");
	if (!capture.isOpened()) {
		printf("could not load video data...\n");
		return -1;
	}


	int frames = capture.get(CAP_PROP_FRAME_COUNT);//获取视频针数目(一帧就是一张图片)
	double fps = capture.get(CAP_PROP_FPS);//获取每针视频的频率
	// 获取帧的视频宽度，视频高度
	Size size = Size(capture.get(CAP_PROP_FRAME_WIDTH), capture.get(CAP_PROP_FRAME_HEIGHT));
	cout << frames << endl;
	cout << fps << endl;
	cout << size << endl;
	// 创建视频中每张图片对象
	Mat frame;


    sensor_msgs::ImagePtr msg;

    ros::Rate loop_rate(10);//以10ms间隔发送图片
    while (nh.ok()) {
        capture >> frame;  

        if (!frame.empty()) {  
            msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();  
            pub.publish(msg);  
        }
        ROS_INFO("runnning!");
        ros::spinOnce();  
        loop_rate.sleep();//与ros::Rate loop_rate相对应,休息10ms
    }
}