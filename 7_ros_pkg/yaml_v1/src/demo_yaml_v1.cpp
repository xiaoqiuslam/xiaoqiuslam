#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <yaml-cpp/yaml.h>

int main(int argc,char** argv)
{
    ros::init(argc, argv, "demo_yaml_v1");
    ros::NodeHandle n("~");

    std::string path;

    //？？？这里不知道怎么写成～开头的路径，等待解答
    n.param<std::string>("path", path, "../config/test.yaml");

    //YAML 读
    YAML::Node config = YAML::LoadFile(path);
    ROS_INFO("name: %s",config["name"].as<std::string>().c_str());
    ROS_INFO("sex: %s",config["sex"].as<std::string>().c_str());
    ROS_INFO("age: %s",config["age"].as<std::string>().c_str());

    //YAML 写
    int age = 456;
    std::ofstream fout(path.c_str());
    config["age"] = age;//添加新元素
    ROS_INFO("write_age: %d",age);
    fout << config;
    fout.close();
    return 0;
}
