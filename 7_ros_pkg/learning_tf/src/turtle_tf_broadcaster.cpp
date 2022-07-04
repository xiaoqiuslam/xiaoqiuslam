#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <turtlesim/Pose.h>

std::string turtle_name;


/**
 * rosmsg info turtlesim/Pose
 * float32 x
 * float32 y
 * float32 theta
 * float32 linear_velocity
 * float32 angular_velocity
 */
void poseCallback(const turtlesim::PoseConstPtr& msg){
  // 定义一个tf::TransformBroadcaster,发布子坐标系turtle_name到父坐标系world到的变换
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  // tf::Transform.setOrigin()设置原点，tf::Vector3(msg->x, msg->y, 0.0)代表的含义是子坐标系turtle_name的原点在父坐标系world下的坐标是（msg->x, msg->y）
  transform.setOrigin( tf::Vector3(msg->x, msg->y, 0.0) );
  tf::Quaternion q;
  // tf::Quaternion.setRPY(roll(绕x轴、翻滚），pitch(绕y轴、俯仰），yaw(绕z轴、偏航）
  q.setRPY(0, 0, msg->theta);
  transform.setRotation(q);
  // tf::StampedTransform(transform变换矩阵,ros::Time::now()系统时间,world为父坐标系名称,turtle_name为子坐标系名称)
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", turtle_name));
}


int main(int argc, char** argv){
  // 初始化ros,创建名字为"my_tf_broadcaster"节点
  ros::init(argc, argv, "my_tf_broadcaster");
  if (argc != 2){
    // 需要在代码运行终端传入一个参数作为小海龟的名字
    ROS_ERROR("need turtle name as argument"); 
    return -1;
  };
  // argv[0]是可执行程序的名字
  // argv[1]是小海龟的名字
  turtle_name = argv[1];

  // 创建节点句柄，不理解请参考文章https://chunqiushenye.blog.csdn.net/article/details/121829632
  ros::NodeHandle node;
  // 节点句柄调用subscribe设置接收的话题名字"turtle_name/pose"这里做了一个字符串拼接
  // 第一个参数指定订阅话题topic；第二个参数设置位姿消息缓冲区的大小；第三个参数指定回调函数也就是位姿处理函数。 
  ros::Subscriber sub = node.subscribe(turtle_name+"/pose", 10, &poseCallback);
  // ros::spin() 将会进入循环，收到10个数据就调用回调函数poseCallback()
  // 当用户输入Ctrl+C或者ROS主进程关闭时退出，
  ros::spin();
  return 0;
};
