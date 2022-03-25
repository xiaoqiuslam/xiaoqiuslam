#include <ros/ros.h>
#include "test.h"

int main(int argc,char** argv)
{
    ros::init(argc, argv, "use_demo_hpp");
    ros::NodeHandle n;
    test one;
    one.working();
    return 0;
}
