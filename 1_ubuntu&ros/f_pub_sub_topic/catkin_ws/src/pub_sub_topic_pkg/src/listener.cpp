#include "ros/ros.h"
#include "std_msgs/String.h"
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}
int main(int argc, char **argv)
{
  // must call ros::init() before using ROS system.
  ros::init(argc, argv, "listener");
  // NodeHandle communications with the ROS system.
  ros::NodeHandle n;
  //subscribe()  receive messages publishing and subscribing.The second parameter size of the message queue.
  // 第一个参数指定订阅话题topic；第二个参数设置消息缓冲区的大小；第三个参数指定回调函数。 
  // ros::Subscriber Class Reference Manages an subscription callback on a specific topic.
  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
  //订阅 chatter 话题上的消息。
  //当有消息发布到这个话题时，ROS 就会调用 chatterCallback() 函数。
  //1000参数是队列大小，当缓存达到 1000 条消息后，再有新的消息到来就将开始丢弃先前接收的消息。
  // Subscribe的第四个参数用法"https://www.bbsmax.com/A/gGdX7KrQ54/"
  /*ros::Subscriber mysub = nh.subscribe("/move_cmd", , &Robot::moveCallback, &tars);
  

    template<class M , class T >
    Subscriber ros::NodeHandle::subscribe(const std::string &        topic,
                                          uint32_t                   queue_size,
                                          void(T::*)(M)              fp,
                                          T *                        obj,
                                          const TransportHints &     transport_hints = TransportHints()
                                          )                          [inline]


  看上面原型要注意的是回调函数，应该是和第四个参数的在同一个类中，上面的例子是都在类T中，并且fp和obj都是指针类型 * 的。

  解释下用法，第四个参数的T* obj是将类T的对象实例的指针传给同类的回调函数，void(T::*)(M)  fp。因此在实例类obj中，如果改了一个变量的值，
  在回 调函数中也能体现出来。这样，就相当于类中使用全局变量了。



    void Robot::moveCallback(const fsm_robot::rcmd_move msg)
    {
      cmd_int = n;
      n++;
    }

  这里，cmd_int就是类实例中中的全局变量了

  指针参数传递本质上是值传递，它所传递的是一个地址值。值传递过程中，被调函数的形式参数作为被调函数的局部变量处理，
  会在栈中开辟内存空间以存放由主调函数传递进来的实参值，从而形成了实参的一个副本（替身）。值传递的特点是，
  被调函数对形式参数的任何操作都是作为局部变量进行的，不会影响主调函数的实参变量的值（形参指针变了，实参指针不会变）。

  引用参数传递过程中，被调函数的形式参数也作为局部变量在栈中开辟了内存空间，但是这时存放的是由主调函数放进来的实参变量的地址。
  被调函数对形参（本体）的任何操作都被处理成间接寻址，即通过栈中存放的地址访问主调函数中的实参变量（根据别名找到主调函数中的本体）。
  因此，被调函数对形参的任何操作都会影响主调函数中的实参变量。*/
  
  ros::spin();
  return 0;
}

