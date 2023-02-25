#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <yaml-cpp/yaml.h>

int main(int argc,char** argv)
{
    // rosrun yaml_cpp yaml_cpp
    ros::init(argc, argv, "yaml_cpp");
    ros::NodeHandle n;
    std::string path;
    n.param<std::string>("path", path, "./src/yaml_cpp/src/test.yaml");

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
