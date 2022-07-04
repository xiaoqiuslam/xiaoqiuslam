#include <ros/ros.h>

int main(int argc,char** argv)
{
    ros::init(argc, argv, "param_demo");
	ros::NodeHandle n;
	ros::NodeHandle pn("~");//roslaunch里面的param自带命名空间，这个"~"使两者对应，从而能改
 
	std::string s;
	int num;
	
	//个人比较喜欢的一种通过roslaunch来传递到常有变量值的方法
	n.param<std::string>("string_param", s, "haha");
	pn.param<int>("int_param", num, 666);
	
	//输出被初始化后的变量值
	ROS_INFO("string_param_init: %s", s.c_str());
	ROS_INFO("int_param_init: %d", num);
 
	//设置参数的值
	n.setParam("string_param", "hehe");
	pn.setParam("int_param", 222);
 
 
	//设置循环的频率为1Hz
	ros::Rate loop_rate(1);	
 
	while(ros::ok())
	{	
		//获取参数的值
		n.getParam("string_param", s);
        pn.getParam("int_param", num);
		
		//输出参数
		ROS_INFO("string_param: %s", s.c_str());
		ROS_INFO("int_param: %d", num);
		
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	return 0;
}