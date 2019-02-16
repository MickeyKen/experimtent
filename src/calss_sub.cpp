
#include "ros/ros.h"
#include "std_msgs/String.h"

/**
 * This tutorial demonstrates subscribing to a topic using a class method as the callback.
 */

// %Tag(CLASS_WITH_DECLARATION)%
class Listener
{
public:
  void callback(const std_msgs::String::ConstPtr& msg);
};
// %EndTag(CLASS_WITH_DECLARATION)%

void Listener::callback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener_class");
  ros::NodeHandle n;

// %Tag(SUBSCRIBER)%
  Listener listener;
  ros::Subscriber sub = n.subscribe("chatter", 1000, &Listener::callback, &listener);
// %EndTag(SUBSCRIBER)%

  ros::spin();

  return 0;
}
