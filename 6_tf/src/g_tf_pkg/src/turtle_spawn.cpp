// 1.包含头文件
#include "ros/ros.h"
#include "turtlesim/Spawn.h"

int main(int argc, char *argv[]){
    setlocale(LC_ALL,"");
    // 2.初始化 ros 节点
    ros::init(argc,argv,"turtle_spawn");
    // 3.创建 ros 句柄
    ros::NodeHandle nh;
    // 4.创建 service 客户端
    ros::ServiceClient client = nh.serviceClient<turtlesim::Spawn>("/spawn");
    // 5.等待服务启动
    // client.waitForExistence();
    ros::service::waitForService("/spawn");
    // 6.发送请求
    turtlesim::Spawn spawn;
    // 位置
    spawn.request.x = 4.0;
    spawn.request.y = 2.0;
    // 朝向
    spawn.request.theta = 0;
    // 名字
    spawn.request.name = "turtle2";
    bool flag = client.call(spawn);
    // 7.处理响应结果
    if (flag){
        ROS_INFO("成功创建一只海龟名字：%s",spawn.response.name.c_str());
    } 
    else {
        ROS_INFO("海龟生成失败");
    }
    return 0;
}