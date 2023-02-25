#include <iostream>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

int main(int argc, char **argv)
{
    // 初始化名字为 ponit_cloud_node 的节点，可用命令 rosrun rqt_graph rqt_graph 查看到
    ros::init(argc, argv, "ponit_cloud_node");
    ros::NodeHandle nh;
    // 建立ros::Publisher，topic是point_cloud_publisher_topic 可用命令 rosrun rqt_graph rqt_graph 查看到
    // The second parameter to advertise() is the size of the message queue used for publishing messages.
    // If messages are published more quickly than we can send them,
    // the number here specifies how many messages to buffer up before throwing some away.
    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("/point_cloud_topic", 1);
    // 点云更新频率3Hz 意思是３ms发布一次点云数据
    ros::Rate rate(3);
    // 建立 pcl 点云
    pcl::PointCloud<pcl::PointXYZRGB> point_cloud;
    // 点云初始化
    // 点云数目为 3
    unsigned int point_size = 3;
    point_cloud.points.resize(point_size);
    // 建立ros点云
    sensor_msgs::PointCloud2 ros_point_cloud;
    // ros::ok() 检查上面创建的节点是否还在
    while (ros::ok()){
        // 调用ros获取时间的接口，获取系统时间
        ros_point_cloud.header.stamp = ros::Time::now();
        // 创建三个绿色的点
        for (int i = 0; i < point_size; i++){
            point_cloud.points[i].x = i;
            point_cloud.points[i].y = i;
            point_cloud.points[i].z = 0;
            point_cloud.points[i].r = 0; 
            point_cloud.points[i].g = 255;
            point_cloud.points[i].b = 0;
        }
        // 将pcl点云转化为ros消息发布
        pcl::toROSMsg(point_cloud, ros_point_cloud);
        // 发布的点云坐标系
        ros_point_cloud.header.frame_id = "point_cloud_link";
        // 发布
        pub.publish(ros_point_cloud);
        rate.sleep();
    }
    ros::spin();

    return 0;
}