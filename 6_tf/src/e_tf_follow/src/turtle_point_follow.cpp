//1.包含头文件
#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"
#include <tf/transform_listener.h>

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    // 2.初始化 ROS 节点
    ros::init(argc,argv,"dynamic_tf_sub");
    ros::NodeHandle nh;


    tf::StampedTransform transform_tf;
    tf::TransformListener listener_tf(ros::Duration(10));


    ros::Publisher turtle_vel = nh.advertise<geometry_msgs::Twist>("turtle2/cmd_vel", 10);

    ros::Rate rate(1);
    while (ros::ok())
    {
    // 4.生成一个坐标点(相对于子级坐标系)
        geometry_msgs::PointStamped turtle1_eye_point;
        turtle1_eye_point.header.frame_id = "turtle1_eye";
        turtle1_eye_point.header.stamp = ros::Time();
        turtle1_eye_point.point.x = 0.1;
        turtle1_eye_point.point.y = 0;
        turtle1_eye_point.point.z = 0.1;
    // 5.转换坐标点(相对于父级坐标系)
        //新建一个坐标点，用于接收转换结果  
        //--------------使用 try 语句或休眠，否则可能由于缓存接收延迟而导致坐标转换失败------------------------
        try
        {
            try{
              listener_tf.waitForTransform("/turtle1", "/turtle1_eye", ros::Time(), ros::Duration(10.0) );
              listener_tf.lookupTransform("/turtle1", "/turtle1_eye", ros::Time(), transform_tf);
              ROS_INFO("子坐标系%s相对于父坐标系%s的坐标关系(%.2f,%.2f,%.2f)",transform_tf.child_frame_id_.c_str(),transform_tf.frame_id_.c_str(),transform_tf.getOrigin().x(), transform_tf.getOrigin().y(), transform_tf.getOrigin().z());
            }
            catch (tf::TransformException &ex) {
              ROS_ERROR("%s",ex.what());
            }

            try{
              listener_tf.waitForTransform("/turtle2", "/turtle1", ros::Time(), ros::Duration(10.0) );
              listener_tf.lookupTransform("/turtle2", "/turtle1", ros::Time(), transform_tf);
              ROS_INFO("子坐标系%s相对于父坐标系%s的坐标关系(%.2f,%.2f,%.2f)",transform_tf.child_frame_id_.c_str(),transform_tf.frame_id_.c_str(),transform_tf.getOrigin().x(), transform_tf.getOrigin().y(), transform_tf.getOrigin().z());
              // turtle2跟随turtle1在动
              geometry_msgs::Twist vel_msg;
              vel_msg.angular.z =  atan2(transform_tf.getOrigin().y(), transform_tf.getOrigin().x());
              vel_msg.linear.x =  sqrt(pow(transform_tf.getOrigin().x(), 2) + pow(transform_tf.getOrigin().y(), 2));
              turtle_vel.publish(vel_msg);
            }
            catch (tf::TransformException &ex) {
              ROS_ERROR("%s",ex.what());
            }

            try{
              listener_tf.waitForTransform("/turtle2", "/turtle1_eye", ros::Time(), ros::Duration(10.0) );
              listener_tf.lookupTransform("/turtle2", "/turtle1_eye", ros::Time(), transform_tf);
              ROS_INFO("子坐标系%s相对于父坐标系%s的坐标关系(%.2f,%.2f,%.2f)",transform_tf.child_frame_id_.c_str(),transform_tf.frame_id_.c_str(),transform_tf.getOrigin().x(), transform_tf.getOrigin().y(), transform_tf.getOrigin().z());
            }
            catch (tf::TransformException &ex) {
              ROS_ERROR("%s",ex.what());
            }

            /**
             * 子坐标系turtle1_eye相对于父坐标系turtle1的坐标关系(0.05,0.00,0.10) +
             * 子坐标系turtle1相对于父坐标系turtle2的坐标关系(7.67,5.54,0.00) =
             * 子坐标系turtle1_eye相对于父坐标系turtle2的坐标关系(7.72,5.54,0.10)
             * 
             * turtle1_eye坐标系下的坐标点相对于turtle1_eye坐标系的坐标为:(0.10,0.00,0.10) + 
             * 子坐标系turtle1_eye相对于父坐标系turtle2的坐标关系(7.72,5.54,0.10) = 
             * turtle1_eye坐标系下的坐标点相对于turtle2坐标系的坐标为:(7.82,5.54,0.20)
             */
            

            geometry_msgs::PointStamped turtle2_point;
            // tf的写法
            listener_tf.transformPoint("turtle2", turtle1_eye_point, turtle2_point);
            ROS_INFO("turtle1_eye坐标系下的坐标点相对于turtle1_eye坐标系的坐标为:(%.2f,%.2f,%.2f)",turtle1_eye_point.point.x,turtle1_eye_point.point.y,turtle1_eye_point.point.z);
            ROS_INFO("turtle1_eye坐标系下的坐标点相对于turtle2坐标系的坐标为:(%.2f,%.2f,%.2f)",turtle2_point.point.x,turtle2_point.point.y,turtle2_point.point.z);


            try{
              listener_tf.waitForTransform("/turtle2_eye", "/turtle2", ros::Time(), ros::Duration(10.0) );
              listener_tf.lookupTransform("/turtle2_eye", "/turtle2", ros::Time(), transform_tf);
              ROS_INFO("子坐标系%s相对于父坐标系%s的坐标关系(%.2f,%.2f,%.2f)",transform_tf.child_frame_id_.c_str(),transform_tf.frame_id_.c_str(),transform_tf.getOrigin().x(), transform_tf.getOrigin().y(), transform_tf.getOrigin().z());
            }
            catch (tf::TransformException &ex) {
              ROS_ERROR("%s",ex.what());
            }

             /**
             * turtle1_eye坐标系下的坐标点相对于turtle1_eye坐标系的坐标为:(0.10,0.00,0.10) +
             * 子坐标系turtle1_eye相对于父坐标系turtle1的坐标关系(0.05,0.00,0.10) + 
             * 子坐标系turtle1相对于父坐标系turtle2的坐标关系(5.59,5.54,0.00) -
             * 子坐标系turtle2相对于父坐标系turtle2_eye的坐标关系(-0.05,0.00,-0.10) =
             * turtle1_eye坐标系下的坐标点相对于turtle2_eye坐标系的坐标为:(5.69,5.54,0.10)
             */

            geometry_msgs::PointStamped turtle2_eye_point;
            // tf的写法
            listener_tf.transformPoint("turtle2_eye", turtle1_eye_point, turtle2_eye_point);
            ROS_INFO("turtle1_eye坐标系下的坐标点相对于turtle1_eye坐标系的坐标为:(%.2f,%.2f,%.2f)",turtle1_eye_point.point.x,turtle1_eye_point.point.y,turtle1_eye_point.point.z);
            ROS_INFO("turtle1_eye坐标系下的坐标点相对于turtle2_eye坐标系的坐标为:(%.2f,%.2f,%.2f)",turtle2_eye_point.point.x,turtle2_eye_point.point.y,turtle2_eye_point.point.z);




        }
        catch(const std::exception& e)
        {
            // std::cerr << e.what() << '\n';
            ROS_INFO("程序异常:%s",e.what());
        }

        rate.sleep();  
        ros::spinOnce();
    }


    return 0;
}
