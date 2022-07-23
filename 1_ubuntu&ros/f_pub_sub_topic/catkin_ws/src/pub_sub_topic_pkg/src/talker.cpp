#include "ros/ros.h" // ROS_HEADER
#include "std_msgs/String.h" // MSG_HEADER
#include <sstream>
// This tutorial demonstrates simple sending of messages over the ROS system.
int main(int argc, char **argv){  
  ros::init(argc, argv, "talker");
  // You must call ros::init() before using any other part of the ROS system.
  // 初始化其中第三个参数是节点名,这个"node_name"可以通过"rosnode list"查看
  // 在ROS系统中必须唯一,如果ROS系统中出现重名则之前的节点会被自动关闭
  // 如果想要多个重名节点而不报错,可以在init中添加ros::init_options::AnonymousName参数
  // ros::init(argc, argv, "my_node_name", ros::init_options::AnonymousName)
  // 该参数会在原有节点名字的后面添加一些随机数来使每个节点独一无二
  ros::NodeHandle n;
  // NodeHandle communications with the ROS system.
  // first NodeHandle initialize this node,last NodeHandle close down the node
  // ros::NodeHandle Class Reference roscpp’s interface for creating subscribers, publishers
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  // advertise() tell ROS you publish a topic name. The second parameter size of the message queue
  // 这个"topic_name"可以通过"rostopic list"查看
  /**
  ROS是一种通信机制的框架，围绕着三种通信机制有很多其他的功能，核心还是三种通信机制
  publishers/subscribers
  services/clients
  action-servers/action-clients
  
  为了确保这种通信机制能够顺利运行，引入了nodes、topic和messages等等的概念，使得这种通信机制更具逻辑性以及便于理解。
  在ROS框架下，可以把一个大项目分成若干个小部分，让不同的人去完成，最后通过ROS这个框架，
  将每个人负责完成的那部分作为一个node，通过在topic上发布和订阅messages，这就实现了各个nodes之间的通信，
  将各个node连接起来，组合完成这个项目。

  publishers/subscribers
  一个node既可以是publishers也可以是subscribers，可以同时发布topic的信息，也可以订阅topic上的信息，
  这里面有一个很重要的特性，一个subscriber只需要知道它要订阅的那个topic的名称，
  并不需要准确知道是哪一个publisher发布的这个topic，同样的，一个publisher也只管发布topic的消息就行了，
  不用管谁会来订阅这个topic上的信息，一个subscriber可以订阅很多个topic上的信息。
  */
  ros::Rate loop_rate(10);
  // 发送消息的频率 10HZ
  int count = 0;
  // count messages have sent. 
  while (ros::ok()){
    // message object. publish it with data.
    std_msgs::String msg;
    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();
    ROS_INFO("%s", msg.data.c_str());
    // publish() messages
    chatter_pub.publish(msg);
    ros::spinOnce();
    /**
    ROS消息回调处理函数,通常会出现在ROS的主循环中，程序需要不断调用ros::spin() 或 ros::spinOnce()，
    两者区别在于前者调用后不会再返回，也就是你的主程序到这儿就不往下执行了，而后者在调用后还可以继续执行之后的程序。
    
    ROS存在消息发布订阅机制，如果你的程序写了相关的消息订阅函数，那么程序在执行过程中，除了主程序以外，
    ROS还会自动在后台按照你规定的格式，接受订阅的消息，但是所接到的消息并不是立刻就被处理，
    而是必须要等到ros::spin()或ros::spinOnce()执行的时候才被调用，这就是消息回到函数的原理。

    就像上面说的
    ros::spin() 在调用后不会再返回，也就是你的主程序到这儿就不往下执行了，
    而 ros::spinOnce() 后者在调用后还可以继续执行之后的程序。

    其实看函数名也能理解个差不多，一个是一直调用；另一个是只调用一次，如果还想再调用，就需要加上循环了。

    ros::spin()函数一般不会出现在循环中，因为程序执行到spin()后就不调用其他语句了，也就是说该循环没有任何意义，
    还有就是spin()函数后面一定不能有其他语句(return 0 除外)，有也是白搭，不会执行的。
    
    ros::spinOnce()的用法相对来说很灵活，但往往需要考虑调用消息的时机，调用频率，以及消息池的大小，
    这些都要根据现实情况协调好，不然会造成数据丢包或者延迟的错误。
    */
    loop_rate.sleep();
    ++count;
  }
  return 0;
}

