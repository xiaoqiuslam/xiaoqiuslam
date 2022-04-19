#include "stdio.h"
#include "iostream" // 输入输出流的方法
#include "string"
#include "thread" // 线程的类、用于互斥访问的类与方法等
#include "mutex" // 提供了多种互斥操作，可以显式避免数据竞争。主要包含mutex类型、lock类型以及功能函数.
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Twist.h"
#include "bsp_Serial.h"
#include "tf/transform_broadcaster.h"
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "turtlesim/Pose.h"

namespace node{
    
    std::mutex mutex_;
    geometry_msgs::Twist twist_; 
    void callBack_vel(const geometry_msgs::TwistConstPtr& msg)    {
        std::lock_guard<std::mutex> lock(mutex_);
        twist_ = *msg;
    }

    uint8_t caVerify_Xor(uint8_t *buf, int startIndex, int endIndexs){
        uint8_t ret = 0;
        int i;

        for(i = startIndex; i < endIndexs; i ++)
        {
            ret ^= buf[i];
        }

        return ret;
    }

    geometry_msgs::Pose pose_;
    // void callBack_pose(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)    {
    //     std::lock_guard<std::mutex> lock(mutex_);
    //     pose_ = msg->pose.pose;
    //     ROS_INFO("小海龟的坐标：(%.2f,　%.2f),　朝向rad：%.2f", pose_.position.x-5.54, pose_.position.y-5.54, pose_.orientation);
    // }

    turtlesim::Pose tur_pose;
    void callBack_pose(const turtlesim::Pose::ConstPtr& pose)    {
        std::lock_guard<std::mutex> lock(mutex_);
        tur_pose.x = pose->x;
        tur_pose.y = pose->y;
        tur_pose.theta = pose->theta;
        ROS_INFO("收到VSLAM的位姿：(%.2f,　%.2f),　朝向rad：%.2f", pose->x, pose->y, pose->theta);
    }

    void callBack_serial_send(const ros::WallTimerEvent& unused_timer_event){
        std::lock_guard<std::mutex> lock(mutex_);
        uint8_t data[100];
        size_t len;
        float temp, temp1;        

        data[0] = 0x06;
        data[1] = 0x27;

        // temp = pose_.position.x;
        *((float*)(&data[4])) = tur_pose.x; 

        // temp = pose_.position.y;
        *((float*)(&data[8])) = tur_pose.y; 

        temp = tur_pose.theta;
        *((float*)(&data[12])) = temp; 

        data[16] = 0;
        data[17] = 0;

        /**
         * @brief 将接收到的线速度信息发送给轮子
         */
        //         布尔表达式?表达式1:表达式2
        // 运算过程：如果布尔表达式的值为 true ，则返回 表达式1 的值，否则返回 表达式2 的值 
        // ROS_INFO("发送给轮子twist_.linear.x：%.2f", twist_.linear.x);
        // temp = twist_.linear.x>=0?1:-1;

        // // 就是输出时四舍五入保留2位小数。若不足2位小数时，补0达到2位小数，整数部分按实际输出
        // // C语言 fabs() 函数用于求双精度浮点数的绝对值 
        // temp1 = (fabs(twist_.linear.x)>0.2f)?0.2f:fabs(twist_.linear.x);
        // temp *= temp1;
        // *((float*)(&data[18])) = temp;

        *((float*)(&data[18])) = twist_.linear.x;

        
        /**
         * @brief 将接收到的角速度信息发送给轮子
         */
        // ROS_INFO("发送给轮子twist_.linear.z：%.2f", twist_.linear.z);
        // temp = (twist_.angular.z>=0)?1:-1;
        // temp1 = (fabs(twist_.angular.z)>0.2f)?0.2f:fabs(twist_.angular.z);
        // temp *= temp1;
        // *((float*)(&data[22])) = temp;

        *((float*)(&data[22])) = twist_.angular.z;
        ROS_INFO("下发送给轮子的速度linear.x：%.2f,angular.z：%.2f", twist_.linear.x, twist_.angular.z);


        data[26] = caVerify_Xor(data,0,26);
        data[27] = 0x0A;
        len = 28;
        serial_write(0, data, len);
    }


    nav_msgs::Odometry odometry; 
    ros::Publisher pub_odometry;
    void callBack_serial_rec(void){     
        uint8_t rxData[100];
        int rxLen =0,index=0;
       
        tf::TransformBroadcaster transform_broadcaster;

        while(ros::ok())
        {
            rxLen = serial_read(0,(rxData+index),(100-index));
            index +=rxLen;
            if(index>20)
            {
                if((index>=30)&&(rxData[0]==0x06)&&(rxData[1]==0x28)&&(rxData[29]==0x0A))
                {
                    float temp = *((float*)&rxData[4]);
                    odometry.pose.pose.position.x = temp;

                    temp = *((float*)&rxData[8]);
                    odometry.pose.pose.position.y = temp;  

                    odometry.pose.pose.position.z = 0;


                    temp = *((float*)&rxData[12]);
                    odometry.pose.pose.orientation = tf::createQuaternionMsgFromYaw(temp);
                    // 弧度 = 角度*M_PI/180
                    // 角度 = 弧度*180/M_PI


                    temp = *((float*)&rxData[16]);
                    odometry.twist.twist.linear.x = temp;  

                    temp = *((float*)&rxData[24]);
                    odometry.twist.twist.angular.z = temp;
                    ROS_INFO("轮子的实时坐标x：%.2f, 坐标y：%.2f, 朝向rad：%.2f", *((float*)&rxData[4]), *((float*)&rxData[8]), *((float*)&rxData[12]));


                    odometry.header.stamp = ros::Time::now();
                    odometry.header.frame_id = "odom";// odom_wheel
                    odometry.child_frame_id = "base_footprint";// base_footprint
    
                    pub_odometry.publish(odometry);


                    // publish the transform over tf
                    // geometry_msgs::TransformStamped transform;
                    // transform.header.stamp = ros::Time::now();;
                    // transform.header.frame_id = "odom";
                    // transform.child_frame_id = "base_footprint";

                    // transform.transform.translation.x = odometry.pose.pose.position.x;
                    // transform.transform.translation.y = odometry.pose.pose.position.y;
                    // transform.transform.translation.z = 0.0;

                    // transform.transform.rotation = odometry.pose.pose.orientation;

                    //send the transform
                    // transform_broadcaster.sendTransform(transform);

                }

                if(index>=30)
                {
                    index = 0;
                }
            }

            if(index>0)
                usleep(10000); 
            else   
                usleep(20000); 
        }
    }

    void run(){
        ros::NodeHandle nh;
        // 1. 接收速度/turtle1/cmd_vel
        // ros::Subscriber sub_twist = nh.subscribe<geometry_msgs::Twist>("/turtle1/cmd_vel", 10, callBack_vel);

        // 2. 接收速度/turtle1/cmd_vel a)运动、b)实时反馈轮子Odometry信息(速度和位置)
        ros::WallTimer timer_serial = nh.createWallTimer(::ros::WallDuration(0.05), callBack_serial_send);
        pub_odometry = nh.advertise<nav_msgs::Odometry>("/wheel_odometry_topic", 10);

        // 3. 接收视觉Odometry信息发送给导航AGVS
        // ros::Subscriber sub_pose = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/turtle1/pose", 10, callBack_pose);
        // ROS_INFO("还没有接收视觉Odometry信息发送给导航AGVS");

        ros::Subscriber sub_pose = nh.subscribe<turtlesim::Pose>("/turtle1/pose", 10, callBack_pose);
        // ROS_INFO("还没有接收视觉Odometry信息发送给导航AGVS");

        
        
        bspSerial_Init();
        bspSerial_Open(0,115200,0,8,1,100); 
        std::thread th(node::callBack_serial_rec);   
        ros::spin();

        // sub_twist.shutdown();

        pub_odometry.shutdown();

        sub_pose.shutdown();
    }

};


int main(int argc,char **argv){
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "wheel_odometry_node");
    ros::start();
    node::run();
    ros::shutdown();
    return 0;
}
