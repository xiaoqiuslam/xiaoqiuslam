#include <ros/ros.h>
//console可以不添加，但是不添加的话，就没得自动显示函数
#include <ros/console.h>
#include <string>

//用于设置打印对应信息
std::string PRINT_MODE;

int main(int argc,char** argv)
{
    ros::init(argc, argv, "demo_log");
    ros::NodeHandle n;
    ros::NodeHandle pn("~");
    pn.param<std::string>("PRINT_MODE",PRINT_MODE,"");

    ros::Rate loop_rate(10);    
    ROS_INFO_STREAM("PRINT_MODE : " << PRINT_MODE);
    while(ros::ok())    
    {
        //INFO可以换成DEBUG，WARN，ERROR，FATAL
        ROS_INFO("printf ROS_INFO");
        ROS_INFO_STREAM("stream"<<" ROS_INFO_STREAM");

        //DEBUG 需要结合console和logger_level调节出来显示，一般都不打印出来
        ROS_DEBUG("printf ROS_DEBUG");
        
        //有条件选择打印啥
        ROS_INFO_COND(PRINT_MODE=="1","printf 1");
        ROS_INFO_STREAM_COND(PRINT_MODE=="2","printf" <<" 2");
        ROS_INFO_STREAM_COND(PRINT_MODE=="3","printf 3");

        loop_rate.sleep();
    }

    return 0;
}