#include <ros/ros.h>  
#include <image_transport/image_transport.h>  
#include <opencv2/highgui/highgui.hpp>  
#include <cv_bridge/cv_bridge.h>  

// 回调函数，当有新的图像消息到达camera/image时该函数就会被调用
void imageCallback(const sensor_msgs::ImageConstPtr& msg)  
{  
	try  
	{  
		// 这段代码用于显示捕捉到的图像
		// 其中cv_bridge::toCvShare(msg, "bgr8")->image
		// 用于将ROS图像消息转化为Opencv支持的图像格式采用bgr8编码方式
		// 和发布节点CvImage(std_msgs::Header(), "bgr8", image).toImageMsg()作用相反
		cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image); 
		cv::waitKey(10);  
	}  
	catch (cv_bridge::Exception& e)  
	{  
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());  
	}  
}  

int main(int argc, char **argv)  
{  
	ros::init(argc, argv, "image_listener");  
	ros::NodeHandle nh;  
	cv::namedWindow("view");  
	cv::startWindowThread();  
	image_transport::ImageTransport it(nh);  
	image_transport::Subscriber sub = it.subscribe("/image", 1, imageCallback);  
	ros::spin();  
	cv::destroyWindow("view");  
}  